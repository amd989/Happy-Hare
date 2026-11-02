# Happy Hare test harness - extruder unload assist.
#
# With an extruder entry sensor, _unload_extruder() reverse-homes off the sensor (or, if the
# sensor already reads empty, assumes the filament is out of the extruder) and stops moving the
# extruder. On an MK3S the IR sensor can read a thin tip as empty while the tip is still in the
# extruder gears, so the MMU gear then pulls against a holding extruder and jams.
#
# extruder_unload_assist (mm, default 0 = off) keeps the extruder turning with the gear for that
# distance after the sensor reads empty, like Prusa's MMU2::HelpUnloadToFinda. The MMU3 profile
# defaults it to 40.
#
# The OFF path must be exactly what it was before the option existed: the expected move lists
# below were captured from the code before this change and must not need editing.
#
#   ./venv/bin/python -m unittest test.test_mmu_extruder_unload_assist
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import logging
import unittest

from test.hh import cfg, profiles, session

logging.getLogger().setLevel(logging.CRITICAL)

FILAMENT_POS_UNLOADED = 0
FILAMENT_POS_LOADED = 10
TIP_AT_GATE = -40.0

# The MMU3 with an MK3S IR sensor as its extruder entry sensor, as on a real MK3S + MMU3
PRUSA_MMU3_IR = profiles.PRUSA_MMU3.derive(
    'prusa_mmu3_ir',
    syms={'MMU_HAS_SENSOR_EXTRUDER': True, 'PIN_EXTRUDER_SENSOR': 'mcu:PA7'},
    description='Prusa MMU3 with an extruder entry (IR) sensor')

GRIP = [(None, 0.8, None, None), (None, -0.8, None, None)] * 3      # idler buzz on re-grip
BOWDEN = ('Fast unloading move through bowden', -658.0, None, None)
GATE_HOME = ('Reverse homing off unit0:mmu_shared_exit sensor', -140.0, 'gear', -1)
PARK = ('Final parking', -23.0, None, None)
EXIT_HOME = ('Reverse homing off extruder sensor', -86.0, 'gear+extruder', -1)

# Captured from the code before extruder_unload_assist existed
BEFORE_SENSOR_TRIGGERED = [EXIT_HOME] + GRIP + [BOWDEN, GATE_HOME, PARK]
BEFORE_SENSOR_EMPTY = GRIP + [BOWDEN, GATE_HOME, PARK]


def rendered_assist(profile):
    parser = cfg.assemble(cfg.render(profile))
    section = next(s for s in parser.sections() if s.startswith('mmu_unit_parameters'))
    return parser.getfloat(section, 'extruder_unload_assist')


class TestDefaults(unittest.TestCase):

    def test_off_by_default_for_other_mmus(self):
        self.assertEqual(rendered_assist(profiles.get('boxturtle')), 0.0)
        self.assertEqual(rendered_assist(profiles.get('tradrack')), 0.0)

    def test_mmu3_defaults_to_40mm(self):
        self.assertEqual(rendered_assist(profiles.get('prusa_mmu3')), 40.0)


class TestUnloadMoves(unittest.TestCase):

    def unload_moves(self, assist, sensor_empty, profile=PRUSA_MMU3_IR):
        """Load gate 0, then return every filament move the unload makes."""
        hh = session(profile)
        self.addCleanup(hh.close)
        hh.boot()
        self.hh = hh
        self.fil = hh.filament()
        hh.place_filament(0, position=TIP_AT_GATE)
        hh.run_gcode('MMU_PRELOAD GATE=0')
        hh.heat_extruder(220)
        hh.mmu.select_gate(0)
        hh.run_gcode('MMU_LOAD')
        self.assertEqual(hh.mmu.filament_pos, FILAMENT_POS_LOADED)
        hh.run_gcode('MMU_TEST_CONFIG UNIT=0 extruder_unload_assist=%s' % assist)

        moves = []
        real_move = hh.mmu.move_filament

        def spy(trace, dist, *args, **kwargs):
            moves.append((trace, round(dist, 2), kwargs.get('motor'), kwargs.get('homing_move')))
            return real_move(trace, dist, *args, **kwargs)

        hh.mmu.move_filament = spy

        if sensor_empty:
            # The MK3S failure: the IR sensor reads empty when the extruder unload starts
            check_sensor = hh.mmu.sensor_manager.check_sensor
            real_unload_extruder = hh.mmu._unload_extruder

            def unload_extruder(*args, **kwargs):
                hh.mmu.sensor_manager.check_sensor = (
                    lambda s: False if s == 'extruder' else check_sensor(s))
                try:
                    return real_unload_extruder(*args, **kwargs)
                finally:
                    hh.mmu.sensor_manager.check_sensor = check_sensor

            hh.mmu._unload_extruder = unload_extruder

        hh.run_gcode('MMU_UNLOAD')
        self.assertEqual(hh.mmu.filament_pos, FILAMENT_POS_UNLOADED)
        self.assertEqual(hh.errors, [])
        return moves

    # -- off: unchanged ----------------------------------------------------------------------

    def test_off_with_sensor_triggered_makes_the_same_moves_as_before(self):
        self.assertEqual(self.unload_moves(0, sensor_empty=False), BEFORE_SENSOR_TRIGGERED)

    def test_off_with_sensor_empty_makes_the_same_moves_as_before(self):
        self.assertEqual(self.unload_moves(0, sensor_empty=True), BEFORE_SENSOR_EMPTY)

    def test_off_parks_where_it_always_did(self):
        self.unload_moves(0, sensor_empty=False)
        park = self.fil.layout['mmu_shared_exit'] + self.hh.mmu.mmu_unit(0).p.gate_parking_distance
        self.assertAlmostEqual(self.fil.tip[0], park, places=1)

    # -- on ----------------------------------------------------------------------------------

    def test_on_with_sensor_empty_drives_the_extruder_out_first(self):
        assist = ('Extruder unload assist', -40.0, 'gear+extruder', None)
        shorter_bowden = (BOWDEN[0], BOWDEN[1] + 40.0, None, None)
        self.assertEqual(self.unload_moves(40, sensor_empty=True),
                         [assist] + GRIP + [shorter_bowden, GATE_HOME, PARK])

    def test_on_with_sensor_triggered_assists_after_homing_off_the_sensor(self):
        assist = ('Extruder unload assist', -25.0, 'gear+extruder', None)
        shorter_bowden = (BOWDEN[0], BOWDEN[1] + 25.0, None, None)
        self.assertEqual(self.unload_moves(25, sensor_empty=False),
                         [EXIT_HOME] + GRIP + [assist, shorter_bowden, GATE_HOME, PARK])

    def test_on_still_parks_at_the_gate(self):
        """The assist distance comes off the bowden move, so the park position is unchanged."""
        self.unload_moves(40, sensor_empty=True)
        park = self.fil.layout['mmu_shared_exit'] + self.hh.mmu.mmu_unit(0).p.gate_parking_distance
        self.assertAlmostEqual(self.fil.tip[0], park, places=1)

    def test_on_does_nothing_without_an_extruder_entry_sensor(self):
        """Box Turtle homes the extruder on filament_compression, not an entry sensor."""
        moves = self.unload_moves(40, sensor_empty=False, profile=profiles.get('boxturtle'))
        self.assertFalse(self.hh.mmu.sensor_manager.has_sensor('extruder'))
        self.assertNotIn('Extruder unload assist', [m[0] for m in moves])


if __name__ == '__main__':
    unittest.main()
