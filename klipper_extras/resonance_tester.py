# A utility class to test resonances of the printer
#
# Copyright (C) 2020  Dmitry Butyugin <dmbutyugin@google.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging, math, os, time, itertools
from . import shaper_calibrate
from subprocess import call

class TestAxis:
    def __init__(self, axis=None, vib_dir=None):
        if axis is None:
            self._name = "axis=%.3f,%.3f" % (vib_dir[0], vib_dir[1])
        else:
            self._name = axis
        if vib_dir is None:
            self._vib_dir = (1., 0.) if axis == 'x' else (0., 1.)
        else:
            s = math.sqrt(sum([d*d for d in vib_dir]))
            self._vib_dir = [d / s for d in vib_dir]
    def matches(self, chip_axis):
        if self._vib_dir[0] and 'x' in chip_axis:
            return True
        if self._vib_dir[1] and 'y' in chip_axis:
            return True
        return False
    def get_name(self):
        return self._name
    def get_point(self, l):
        return (self._vib_dir[0] * l, self._vib_dir[1] * l)

def _parse_axis(gcmd, raw_axis):
    if raw_axis is None:
        return None
    raw_axis = raw_axis.lower()
    if raw_axis in ['x', 'y']:
        return TestAxis(axis=raw_axis)
    dirs = raw_axis.split(',')
    if len(dirs) != 2:
        raise gcmd.error("""{"code": "key304", "msg": "Invalid format of axiss '%s'", "values":["%s"]}""" % (raw_axis,raw_axis))
    try:
        dir_x = float(dirs[0].strip())
        dir_y = float(dirs[1].strip())
    except:
        raise gcmd.error(
                """{"code": "key305", "msg": "Unable to parse axis direction '%s'", "values":["%s"]}""" % (raw_axis, raw_axis))
    return TestAxis(vib_dir=(dir_x, dir_y))

class VibrationPulseTestGenerator:
    def __init__(self, config):
        self.min_freq = config.getfloat('min_freq', 5., minval=1.)
        self.max_freq = config.getfloat('max_freq', 10000. / 75.,
                                        minval=self.min_freq, maxval=200.)
        self.accel_per_hz = config.getfloat('accel_per_hz', 75., above=0.)
        self.hz_per_sec = config.getfloat('hz_per_sec', 1.,
                                          minval=0.1, maxval=2.)
    def prepare_test(self, gcmd):
        self.freq_start = gcmd.get_float("FREQ_START", self.min_freq, minval=1.)
        self.freq_end = gcmd.get_float("FREQ_END", self.max_freq,
                                       minval=self.freq_start, maxval=200.)
        self.test_accel_per_hz = self.accel_per_hz
        self.test_hz_per_sec = gcmd.get_float("HZ_PER_SEC", self.hz_per_sec,
                                              above=0., maxval=2.)
    def gen_test(self):
        freq = self.freq_start
        res = []
        sign = 1.
        time = 0.
        while freq <= self.freq_end + 0.000001:
            t_seg = .25 / freq
            accel = self.test_accel_per_hz * freq
            time += t_seg
            res.append((time, sign * accel, freq))
            time += t_seg
            res.append((time, -sign * accel, freq))
            freq += 2. * t_seg * self.test_hz_per_sec
            sign = -sign
        return res

class SweepingVibrationsTestGenerator:
    def __init__(self, config):
        self.vibration_generator = VibrationPulseTestGenerator(config)
        self.sweeping_accel = config.getfloat('sweeping_accel', 400., above=0.)
        self.sweeping_period = config.getfloat('sweeping_period', 1.2,
                                               minval=0.)
    def prepare_test(self, gcmd):
        self.vibration_generator.prepare_test(gcmd)
        self.test_sweeping_accel = gcmd.get_float("SWEEPING_ACCEL", 
                                                 self.sweeping_accel, above=0.)
        self.test_sweeping_period = gcmd.get_float("SWEEPING_PERIOD", 
                                                  self.sweeping_period, minval=0.)
    def gen_test(self):
        test_seq = self.vibration_generator.gen_test()
        accel_fraction = math.sqrt(2.0) * 0.125
        if self.test_sweeping_period:
            t_rem = self.test_sweeping_period * accel_fraction
            sweeping_accel = self.test_sweeping_accel
        else:
            t_rem = float('inf')
            sweeping_accel = 0.
        res = []
        last_t = 0.
        sig = 1.
        accel_fraction += 0.25
        for next_t, accel, freq in test_seq:
            t_seg = next_t - last_t
            while t_rem <= t_seg:
                last_t += t_rem
                res.append((last_t, accel, freq, sweeping_accel * sig))
                t_seg -= t_rem
                t_rem = self.test_sweeping_period * accel_fraction
                accel_fraction = 0.5
                sig = -sig
            t_rem -= t_seg
            res.append((next_t, accel, freq, sweeping_accel * sig))
            last_t = next_t
        return res

class VibrationPulseTest:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object('gcode')
        self.generator = SweepingVibrationsTestGenerator(config)
        self.probe_points = config.getlists('probe_points', seps=(',', '\n'),
                                            parser=float, count=3)
        self.low_mem = config.getboolean('low_mem', True)

    def get_start_test_points(self):
        return self.probe_points
    def prepare_test(self, gcmd):
        self.generator.prepare_test(gcmd)
    def run_test(self, axis, gcmd):
        toolhead = self.printer.lookup_object('toolhead')
        X, Y, Z, E = toolhead.get_position()
        
        toolhead_info = toolhead.get_status(self.printer.get_reactor().monotonic())
        old_max_accel = toolhead_info['max_accel']
        old_max_accel_to_decel = toolhead_info['max_accel_to_decel']
        
        test_seq = self.generator.gen_test()
        max_test_accel = max([abs(a) + abs(s) for _, a, _, s in test_seq])
        
        self.gcode.run_script_from_command(
                "SET_VELOCITY_LIMIT ACCEL=%.3f ACCEL_TO_DECEL=%.3f" % (
                    max_test_accel + 5.0, max_test_accel + 5.0))
                    
        input_shaper = self.printer.lookup_object('input_shaper', None)
        if input_shaper is not None and not gcmd.get_int('INPUT_SHAPING', 0):
            input_shaper.disable_shaping()
            gcmd.respond_info("Disabled [input_shaper] for resonance testing")
        else:
            input_shaper = None

        last_t = last_v_sweep = last_freq = 0.
        sweep_dist = 0.
        for next_t, accel, freq, s_accel in test_seq:
            t_seg = next_t - last_t
            
            v_sweep = last_v_sweep + s_accel * t_seg
            sweep_dist += (v_sweep + last_v_sweep) * 0.5 * t_seg
            last_v_sweep = v_sweep
            
            max_v = abs(accel * t_seg) + abs(v_sweep)
            
            L = (accel * t_seg * t_seg)
            dX, dY = axis.get_point(L)
            sX, sY = axis.get_point(sweep_dist)
            
            nX, nY = X + sX + dX, Y + sY + dY
            
            toolhead.move([nX, nY, Z, E], max_v)
            
            if math.floor(freq) > math.floor(last_freq):
                gcmd.respond_info("Testing frequency %.0f Hz" % (freq,))
                last_freq = freq
            last_t = next_t

        toolhead.wait_moves()
        self.gcode.run_script_from_command(
                "SET_VELOCITY_LIMIT ACCEL=%.3f ACCEL_TO_DECEL=%.3f" % (
                    old_max_accel, old_max_accel_to_decel))
        if input_shaper is not None:
            input_shaper.enable_shaping()

class ResonanceTester:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.move_speed = config.getfloat('move_speed', 50., above=0.)
        self.test = VibrationPulseTest(config)
        if not config.get('accel_chip_x', None):
            self.accel_chip_names = [('xy', config.get('accel_chip').strip())]
        else:
            self.accel_chip_names = [
                ('x', config.get('accel_chip_x').strip()),
                ('y', config.get('accel_chip_y').strip())]
            if self.accel_chip_names[0][1] == self.accel_chip_names[1][1]:
                self.accel_chip_names = [('xy', self.accel_chip_names[0][1])]
        
        self.max_smoothing = config.getfloat('max_smoothing', None, minval=0.05)
        self.gcode = self.printer.lookup_object('gcode')
        self.gcode.register_command("MEASURE_AXES_NOISE", self.cmd_MEASURE_AXES_NOISE)
        self.gcode.register_command("TEST_RESONANCES", self.cmd_TEST_RESONANCES)
        self.gcode.register_command("SHAPER_CALIBRATE", self.cmd_SHAPER_CALIBRATE)
        self.printer.register_event_handler("klippy:connect", self.connect)

    def connect(self):
        self.accel_chips = [
                (chip_axis, self.printer.lookup_object(chip_name))
                for chip_axis, chip_name in self.accel_chip_names]

    def _run_test(self, gcmd, axes, helper, raw_name_suffix=None,
                  accel_chips=None, test_point=None):
        toolhead = self.printer.lookup_object('toolhead')
        calibration_data = {axis: None for axis in axes}
        self.test.prepare_test(gcmd)
        test_points = [test_point] if test_point else self.test.get_start_test_points()

        for point in test_points:
            toolhead.manual_move(point, self.move_speed)
            for axis in axes:
                toolhead.wait_moves()
                toolhead.dwell(0.500)
                raw_values = []
                if accel_chips is None:
                    for chip_axis, chip in self.accel_chips:
                        if axis.matches(chip_axis):
                            raw_values.append((chip_axis, chip.start_internal_client(), chip.name))
                else:
                    for chip in accel_chips:
                        raw_values.append((axis, chip.start_internal_client(), chip.name))

                self.test.run_test(axis, gcmd)
                
                for chip_axis, aclient, chip_name in raw_values:
                    aclient.finish_measurements()
                    if raw_name_suffix is not None:
                        raw_name = self.get_filename('raw_data', raw_name_suffix, axis,
                                point if len(test_points) > 1 else None,
                                chip_name if accel_chips is not None else None,)
                        aclient.write_to_file(raw_name)

                if helper is None: continue
                for chip_axis, aclient, chip_name in raw_values:
                    if not aclient.has_valid_samples():
                        raise gcmd.error("""{"code":"key56", "msg":"accelerometer '%s' measured no data", "values": ["%s"]}""" % (chip_name, chip_name))
                    new_data = helper.lowmem_process_accelerometer_data(aclient) if self.test.low_mem else helper.process_accelerometer_data(aclient)
                    calibration_data[axis] = new_data if calibration_data[axis] is None else calibration_data[axis].add_data(new_data)
        return calibration_data

    def cmd_TEST_RESONANCES(self, gcmd):
        axis = _parse_axis(gcmd, gcmd.get("AXIS").lower())
        accel_chips = gcmd.get("CHIPS", None)
        test_point = gcmd.get("POINT", None)
        if test_point:
            test_point = [float(p.strip()) for p in test_point.split(',')]

        parsed_chips = None
        if accel_chips:
            parsed_chips = []
            for chip_name in accel_chips.split(','):
                cname = chip_name.strip()
                chip_lookup_name = cname if "adxl345" in cname else "adxl345 " + cname
                parsed_chips.append(self.printer.lookup_object(chip_lookup_name))

        outputs = gcmd.get("OUTPUT", "resonances").lower().split(',')
        name_suffix = gcmd.get("NAME", time.strftime("%Y%m%d_%H%M%S"))
        csv_output = 'resonances' in outputs
        raw_output = 'raw_data' in outputs
        helper = shaper_calibrate.ShaperCalibrate(self.printer) if csv_output else None

        data = self._run_test(gcmd, [axis], helper, name_suffix if raw_output else None,
                              parsed_chips if accel_chips else None, test_point)[axis]
        if csv_output:
            csv_name = self.save_calibration_data('resonances', name_suffix, helper, axis, data, point=test_point)
            gcmd.respond_info("Resonances data written to %s" % (csv_name,))

    def cmd_SHAPER_CALIBRATE(self, gcmd):
        axis = gcmd.get("AXIS", None)
        copy_y_to_x = False
        if not axis: calibrate_axes = [TestAxis('x'), TestAxis('y')]
        else:
            calibrate_axes = [TestAxis(axis.lower())]
            if axis.lower() == "y": copy_y_to_x = True

        max_smoothing = gcmd.get_float("MAX_SMOOTHING", self.max_smoothing, minval=0.05)
        name_suffix = gcmd.get("NAME", time.strftime("%Y%m%d_%H%M%S"))
        helper = shaper_calibrate.ShaperCalibrate(self.printer)
        calibration_data = self._run_test(gcmd, calibrate_axes, helper)
        configfile = self.printer.lookup_object('configfile')

        for axis in calibrate_axes:
            axis_name = axis.get_name()
            calibration_data[axis].normalize_to_frequencies()
            best_shaper, all_shapers = helper.find_best_shaper(calibration_data[axis], max_smoothing, gcmd.respond_info)
            gcmd.respond_info("Recommended shaper_type_%s = %s, shaper_freq_%s = %.1f Hz" % (axis_name, best_shaper.name, axis_name, best_shaper.freq))
            helper.save_params(configfile, axis_name, best_shaper.name, best_shaper.freq)
            self.save_calibration_data('calibration_data', name_suffix, helper, axis, calibration_data[axis], all_shapers)
            if copy_y_to_x:
                helper.save_params(configfile, "x", best_shaper.name, best_shaper.freq)
                self.save_calibration_data('calibration_data', name_suffix, helper, TestAxis('x'), calibration_data[axis], all_shapers)

        self.gcode.run_script_from_command("CXSAVE_CONFIG")
        call("sync", shell=True)
        input_shaper = self.printer.lookup_object("input_shaper", None)
        if not input_shaper:
            config = configfile.read_main_config()
            self.printer.reload_object(config, "input_shaper")
            self.gcode.run_script_from_command("UPDATE_INPUT_SHAPER")
            input_shaper.enable_shaping()
        gcmd.respond_info("SAVE_CONFIG command updated config and restarted.")

    def cmd_MEASURE_AXES_NOISE(self, gcmd):
        meas_time = gcmd.get_float("MEAS_TIME", 2.)
        raw_values = [(ax, chip.start_internal_client()) for ax, chip in self.accel_chips]
        self.printer.lookup_object('toolhead').dwell(meas_time)
        for ax, aclient in raw_values: aclient.finish_measurements()
        helper = shaper_calibrate.ShaperCalibrate(self.printer)
        for ax, aclient in raw_values:
            data = helper.process_accelerometer_data(aclient)
            gcmd.respond_info("Noise %s: %.6f(x), %.6f(y), %.6f(z)" % (ax, data.psd_x.mean(), data.psd_y.mean(), data.psd_z.mean()))

    def is_valid_name_suffix(self, name_suffix):
        return name_suffix.replace('-', '').replace('_', '').isalnum()

    def get_filename(self, base, name_suffix, axis=None, point=None, chip_name=None):
        name = base
        if axis: name += '_' + axis.get_name()
        if chip_name: name += '_' + chip_name.replace(" ", "_")
        if point: name += "_%.3f_%.3f_%.3f" % (point[0], point[1], point[2])
        name += '_' + name_suffix
        return os.path.join("/tmp", name + ".csv")

    def save_calibration_data(self, base, suffix, helper, axis, data, all_shapers=None, point=None):
        out = self.get_filename(base, suffix, axis, point)
        helper.save_calibration_data(out, data, all_shapers)
        return out

def load_config(config):
    return ResonanceTester(config)