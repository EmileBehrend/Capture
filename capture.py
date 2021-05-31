import json
import os
import serial
import serial.tools.list_ports
import signal
import struct
import sys
import threading
import time

import cv2
import numpy as np

import arducam_config_parser
import ArducamSDK
import ImageConvert

sys.setrecursionlimit(10 ** 9)

settings_file = "recording_settings.json"

running = True
save_multiview = None
save_single_flag = False
save_flag = False
save_beginning = 0
save_raw = False
calibrate_flag = True
cfg = {}
handle = {}

screen_width = 1200
screen_height = screen_width / 4 * 3

resize_ratio = float(1200) / float(3664)
rectangle_width = float(916) * resize_ratio
rectangle_height = float(686) * resize_ratio

print("Screen size: %d * %d." % (screen_width, screen_height))
print("Rectangle size: %d * %d." % (rectangle_width, rectangle_height))

select = None
background_image = None
move_rectangle = False
mouse_x = 0
mouse_y = 0
draw_x = None
draw_y = None

coarse_integration = 400

LED_MIDDLE = 12
LED_ORDER = [0, 2, 4,
             6, 8, 10,
             12, 13, 15]
LED_AVAILABLE = 16
LED_MAX_ITERATIONS = len(LED_ORDER) * 4 - 1
LED_DROP = 0

calibrate_flag = None
calibrate_start = 200
calibrate_increase = 50
calibrate_threshold = 5
calibrate_cap = 6000
calibrate_grey = None
calibrate_target = None
calibrate_color = None
calibrate_at = calibrate_start
calibrate_results = {}

arduino = None


def arduino_write_read(target, color, display=False):
    if arduino is None:
        print("WARNING: no Arduino was found!")
    else:
        arduino.write(struct.pack(">h", color * LED_AVAILABLE + target))
        arduino.flush()

        result = arduino.readline().decode("utf-8").strip()

        if display:
            print(result)

        return result


def get_multiview_components(input):
    target = LED_ORDER[input % len(LED_ORDER)]
    color = input // len(LED_ORDER)

    return target, color


def is_digit(n):
    try:
        int(n)
        return True
    except ValueError:
        return False


def inborders(n, min_value, max_value):
    return max(min(n, max_value), min_value)


def sigint_handler(signum, frame):
    global running, handle
    running = False
    exit()


signal.signal(signal.SIGINT, sigint_handler)
signal.signal(signal.SIGTERM, sigint_handler)


def configBoard(config):
    global handle
    ArducamSDK.Py_ArduCam_setboardConfig(handle, config.params[0],
                                         config.params[1], config.params[2], config.params[3],
                                         config.params[4:config.params_length])


def camera_initFromFile(fileName, p_width=None, p_height=None):
    global cfg, handle, width, height, color_mode, save_raw
    config = arducam_config_parser.LoadConfigFile(fileName)

    camera_parameter = config.camera_param.getdict()

    if p_width is None:
        width = camera_parameter["WIDTH"]
    else:
        width = p_width

    if p_height is None:
        height = camera_parameter["HEIGHT"]
    else:
        height = p_height

    BitWidth = camera_parameter["BIT_WIDTH"]
    ByteLength = 1
    if BitWidth > 8 and BitWidth <= 16:
        ByteLength = 2
        save_raw = True
    FmtMode = camera_parameter["FORMAT"][0]
    color_mode = camera_parameter["FORMAT"][1]
    print("Color mode: %d." % (color_mode))

    I2CMode = camera_parameter["I2C_MODE"]
    I2cAddr = camera_parameter["I2C_ADDR"]
    TransLvl = camera_parameter["TRANS_LVL"]
    cfg = {
        "u32CameraType": 0x00,
        "u32Width": width,
        "u32Height": height,
        "usbType": 0,
        "u8PixelBytes": ByteLength,
        "u16Vid": 0,
        "u32Size": 0,
        "u8PixelBits": BitWidth,
        "u32I2cAddr": I2cAddr,
        "emI2cMode": I2CMode,
        "emImageFmtMode": FmtMode,
        "u32TransLvl": TransLvl
    }

    ret, handle, rtn_cfg = ArducamSDK.Py_ArduCam_autoopen(cfg)
    if ret == 0:
        usb_version = rtn_cfg['usbType']
        configs = config.configs
        configs_length = config.configs_length
        for i in range(configs_length):
            type = configs[i].type
            if ((type >> 16) & 0xFF) != 0 and ((type >> 16) & 0xFF) != usb_version:
                continue
            if type & 0xFFFF == arducam_config_parser.CONFIG_TYPE_REG:
                ArducamSDK.Py_ArduCam_writeSensorReg(
                    handle, configs[i].params[0], configs[i].params[1])
            elif type & 0xFFFF == arducam_config_parser.CONFIG_TYPE_DELAY:
                time.sleep(float(configs[i].params[0]) / 1000)
            elif type & 0xFFFF == arducam_config_parser.CONFIG_TYPE_VRCMD:
                configBoard(configs[i])

        rtn_val, datas = ArducamSDK.Py_ArduCam_readUserData(
            handle, 0x400 - 16, 16)
        print("Serial: %c%c%c%c-%c%c%c%c-%c%c%c%c." % (datas[0], datas[1], datas[2], datas[3],
                                                       datas[4], datas[5], datas[6], datas[7],
                                                       datas[8], datas[9], datas[10], datas[11]))

        return True, handle
    else:
        print("Failed to open, return value: %s." % (ret))
        return False, handle


def captureImage_thread():
    global handle, running

    rtn_val = ArducamSDK.Py_ArduCam_beginCaptureImage(handle)
    if rtn_val != 0:
        print("Failed to begin capture, return value: %s." % (rtn_val))
        running = False
        return
    else:
        print("Capture began: %s." % (rtn_val))

    while running:
        # print "capture"
        rtn_val = ArducamSDK.Py_ArduCam_captureImage(handle)
        if rtn_val > 255:
            print("Error while capturing image: %s." % (rtn_val))
            if rtn_val == ArducamSDK.USB_CAMERA_USB_TASK_ERROR:
                break
        time.sleep(0.005)

    running = False
    ArducamSDK.Py_ArduCam_endCaptureImage(handle)


def capture_background():
    global handle, running, width, height, cfg, color_mode, select, background_image

    while ArducamSDK.Py_ArduCam_availableImage(handle) == 0:
        time.sleep(0.001)

    rtn_val, data, rtn_cfg = ArducamSDK.Py_ArduCam_readImage(handle)
    datasize = rtn_cfg['u32Size']
    if rtn_val != 0 or datasize == 0:
        ArducamSDK.Py_ArduCam_del(handle)
        print("Failed to read data!")
        exit()

    ArducamSDK.Py_ArduCam_del(handle)

    background_image = ImageConvert.convert_image(data, rtn_cfg, color_mode)
    background_image = cv2.resize(background_image, (screen_width, screen_height), interpolation=cv2.INTER_LINEAR)
    return background_image


def readImage_thread():
    global handle, running, width, height, save_multiview, LED_DROP, save_single_flag, save_flag, save_beginning, save_raw, cfg, color_mode, \
           calibrate_flag, calibrate_grey, calibrate_at, calibrate_target, calibrate_color, calibrate_results

    time0 = time.time()
    time1 = time.time()
    single_count = 0
    count = 0
    totalFrame = 0

    data = {}

    cv2.namedWindow("ArduCam output", 1)

    if not os.path.exists("images"):
        os.makedirs("images")

    while running:
        if ArducamSDK.Py_ArduCam_availableImage(handle) > 0:
            rtn_val, data, rtn_cfg = ArducamSDK.Py_ArduCam_readImage(handle)
            datasize = rtn_cfg['u32Size']
            if rtn_val != 0 or datasize == 0:
                ArducamSDK.Py_ArduCam_del(handle)
                print("Failed to read data.")
                continue

            image = ImageConvert.convert_image(data, rtn_cfg, color_mode)

            time1 = time.time()
            interval = 3  # compute framerate every [interval] seconds
            if time1 - time0 >= interval:
                if save_flag:
                    elapsed_time = time.time() - save_beginning

                    print("Frames per second: %d/s (#%d to #%d), elapsed time: %ds." % (count / interval, totalFrame - count, totalFrame, elapsed_time))
                else:
                    print("Frames per second: %d/s." % (count / interval))

                count = 0
                time0 = time1

            count += 1

            if LED_DROP > 0:
                LED_DROP -= 1
            else:
                if calibrate_flag is not None:
                    if calibrate_grey is None:  # first pass
                        calibrate_grey = np.median(image)
                        print("Target grey: %f." % (calibrate_grey))
                    elif calibrate_flag >= 0:
                        target, color = get_multiview_components(calibrate_flag)

                        if target != calibrate_target or color != calibrate_color:
                            calibrate_target = target
                            calibrate_color = color

                            arduino_write_read(target, color)
                            ArducamSDK.Py_ArduCam_writeSensorReg(handle, 0x0202, calibrate_at)

                            LED_DROP = 1
                        else:
                            current_grey = np.median(image)

                            if current_grey + calibrate_threshold >= calibrate_grey or calibrate_at >= calibrate_cap:
                                print("Calibration result for led %d and color %d (raw: %d): %d (current grey: %f)." % (target, color, calibrate_flag, calibrate_at, current_grey))

                                if target not in calibrate_results.keys():
                                    calibrate_results[target] = {color: calibrate_at}
                                else:
                                    calibrate_results[target][color] = calibrate_at

                                calibrate_at = calibrate_start

                                calibrate_flag -= 1
                            else:
                                calibrate_at += calibrate_increase  # TODO: adaptative increase

                                ArducamSDK.Py_ArduCam_writeSensorReg(handle, 0x0202, calibrate_at)

                                LED_DROP = 1
                    else:
                        print("Middle led: %d." % (LED_MIDDLE))
                        arduino_write_read(LED_MIDDLE, 0)
                        ArducamSDK.Py_ArduCam_writeSensorReg(handle, 0x0202, coarse_integration)
                        LED_DROP = 1

                        print("Calibration finished.")

                        calibrate_flag = None
                elif save_multiview is not None:
                    previous = save_multiview + 1

                    if save_multiview >= 0:
                        target, color = get_multiview_components(save_multiview)

                        if target in calibrate_results.keys():
                            integration = calibrate_results[target][color]
                        else:
                            integration = coarse_integration

                        arduino_write_read(target, color)
                        ArducamSDK.Py_ArduCam_writeSensorReg(handle, 0x0202, integration)
                        LED_DROP = 1

                        if save_multiview != LED_MAX_ITERATIONS:  # first pass
                            target, color = get_multiview_components(previous)  # previous

                            if target in calibrate_results.keys():
                                integration = calibrate_results[target][color]
                            else:
                                integration = coarse_integration

                            grey = np.median(image)

                            cv2.imwrite("images/_multi_%d_%d.png" % (target, color), image)
                            print("Multiview image with led %d and color %d saved (raw: %d), with integration time: %d (grey: %f)." % (target, color, previous, integration, grey))

                        save_multiview -= 1
                    else:
                        print("Middle led: %d." % (LED_MIDDLE))
                        arduino_write_read(LED_MIDDLE, 0)
                        ArducamSDK.Py_ArduCam_writeSensorReg(handle, 0x0202, coarse_integration)  # reset integration to base value
                        LED_DROP = 1

                        target, color = get_multiview_components(previous)  # previous

                        cv2.imwrite("images/_multi_%d_%d.png" % (target, color), image)
                        print("Multiview image with led %d and color %d saved (raw: %d)." % (target, color, previous))

                        save_multiview = None
                else:
                    if save_single_flag:
                        cv2.imwrite("images/_single%d.png" % single_count, image)
                        print("Single image #%d saved." % (single_count))

                        single_count += 1

                        save_single_flag = False

                    if save_flag:
                        cv2.imwrite("images/image%d.png" % totalFrame, image)

                        if save_raw:
                            with open("images/image%d.raw" % totalFrame, 'wb') as f:
                                f.write(data)

                        totalFrame += 1

            image = cv2.resize(image, (width, height), interpolation=cv2.INTER_LINEAR)

            cv2.imshow("ArduCam output", image)
            cv2.waitKey(10)
            ArducamSDK.Py_ArduCam_del(handle)
        else:
            time.sleep(0.001)


def draw_rectangle(output=True):
    global select, background_image, draw_x, draw_y

    if draw_x is None or draw_y is None:
        return

    blank = background_image.copy()
    cv2.rectangle(blank,
                  (
                      draw_x - int(rectangle_width / float(2)),
                      draw_y - int(rectangle_height / float(2))
                  ),
                  (
                      draw_x + int(rectangle_width / float(2)),
                      draw_y + int(rectangle_height / float(2))
                  ),
                  (0, 0, 255),
                  1
                  )

    cv2.imshow(select, blank)
    return blank


def mouse(event, x, y, flags, params):
    global select, background_image, move_rectangle, mouse_x, mouse_y, draw_x, draw_y
    x = inborders(x, int(rectangle_width / 2), screen_width - int(rectangle_width / 2) - 1)
    y = inborders(y, int(rectangle_height / 2), screen_height - int(rectangle_height / 2) - 1)

    shifted_x = - x + screen_width
    shifted_y = y

    display_rectangle = False

    if event == cv2.EVENT_LBUTTONDOWN:
        move_rectangle = True

    elif event == cv2.EVENT_MOUSEMOVE:
        if move_rectangle:
            mouse_x = shifted_x
            mouse_y = shifted_y
            draw_x = x
            draw_y = y
            display_rectangle = True

    elif event == cv2.EVENT_LBUTTONUP:
        move_rectangle = False

        mouse_x = shifted_x
        mouse_y = shifted_y
        draw_x = x
        draw_y = y
        display_rectangle = True

    if display_rectangle:
        display_rectangle = False
        draw_rectangle()


def get_focus():
    global handle, running, width, height, cfg, color_mode, select, background_image

    select = "Zone selection"
    cv2.namedWindow(select)
    cv2.setMouseCallback(select, mouse)

    exited = False
    while True:
        background_image = capture_background()
        overview = draw_rectangle()

        key = cv2.waitKey(10)

        if key == 13:  # enter
            break
        elif key == 27 & 0xFF:  # escape
            exited = True
            break

    result = not exited and mouse_x is not None and mouse_y is not None
    if result:
        cv2.imwrite("images/_overview.png", overview)

    cv2.destroyAllWindows()

    return result


def show_help():
    print("Usage: python capture.py	\
        \nWhile the program is running, you can enter the following inputs in the terminal:	\
        \n 'h' + Enter: Display this help message.	\
        \n 'q' + Enter: Exit the program.	\
        \n 't' + Enter: Save a single image.	\
        \n 's' + Enter: Start the recording.	\
        \n 'c' + Enter: Stop the recording.	\
        \n 'l' + Enter: Move the view to the left.	\
        \n 'r' + Enter: Move the view to the right.	\
        \n 'u' + Enter: Move the view up.	\
        \n 'd' + Enter: Move the view down.	\
        \n 'w' + Enter: Increase coarse_integration time.	\
        \n 'b' + Enter: Decrease coarse_integration time.	\
        \n 'p' + Enter: Save the current parameters.	\
        \n '[number]' + Enter: Change the shifting value to [number].	\
        ")


if __name__ == "__main__":
    for port in list(serial.tools.list_ports.comports()):
        if "Arduino" in port.description:
            arduino = serial.Serial(port=port.device, baudrate=115200, timeout=1)

            break

    try:
        arduino_write_read(0, 0)  # Wait for the Arduino to be initialized
    except Exception:
        pass

    time.sleep(1)

    show_help()

    config_overview = "./config/3664_2748.cfg"
    config_focused = "./config/916_686.cfg"

    ret, handle = camera_initFromFile(config_overview)
    if ret:
        ArducamSDK.Py_ArduCam_setMode(handle, ArducamSDK.CONTINUOUS_MODE)
        ArducamSDK.Py_ArduCam_writeSensorReg(handle, 0x0202, 200)

        print("Middle led: %d." % (LED_MIDDLE))
        arduino_write_read(LED_MIDDLE, 0)

        ct = threading.Thread(target=captureImage_thread)
        ct.start()

        parameters = None
        if os.path.exists(settings_file):
            with open(settings_file) as json_file:
                parameters = json.load(json_file)

                mouse_x = parameters["mouse_x"]
                mouse_y = parameters["mouse_y"]
                draw_x = parameters["draw_x"]
                draw_y = parameters["draw_y"]

                print("Loaded parameters - mouse: (%d; %d), draw: (%d; %d)." % (mouse_x, mouse_y, draw_x, draw_y))

        result = get_focus()

        top_right_x = int(mouse_x - rectangle_width / 2)
        top_right_y = int(mouse_y - rectangle_height / 2)

        running = False
        ct.join()

        ArducamSDK.Py_ArduCam_close(handle)

        if not result:
            exit()

        print("Final parameters - mouse: (%d; %d), draw: (%d; %d)." % (mouse_x, mouse_y, draw_x, draw_y))

        time.sleep(1)

        running = True

        ret, handle = camera_initFromFile(config_focused)

        ct = threading.Thread(target=captureImage_thread)
        rt = threading.Thread(target=readImage_thread)
        ct.start()
        rt.start()

        ArducamSDK.Py_ArduCam_writeSensorReg(handle, 0x0206, 146 / 4)  # analogue gain greenr
        ArducamSDK.Py_ArduCam_writeSensorReg(handle, 0x0208, 162 / 4)  # analogue gain  red
        ArducamSDK.Py_ArduCam_writeSensorReg(handle, 0x020a, 168 / 4)  # analogue gain  blue
        ArducamSDK.Py_ArduCam_writeSensorReg(handle, 0x020c, 144 / 4)  # analogue gain  greenb

        ArducamSDK.Py_ArduCam_writeSensorReg(handle, 0x020E, 256)  # digital gain greenr
        ArducamSDK.Py_ArduCam_writeSensorReg(handle, 0x0210, 256)  # digital gain red
        ArducamSDK.Py_ArduCam_writeSensorReg(handle, 0x0212, 256)  # digital gain blue
        ArducamSDK.Py_ArduCam_writeSensorReg(handle, 0x0214, 256)  # digital gain greenb

        if parameters is not None:
            coarse_integration = parameters["coarse_integration"]
            print("Loaded coarse_integration: %d." % (coarse_integration))

            calibrate_results = {}
            raw_calibration = parameters["calibration"]
            for target, colors in raw_calibration.items():
                entry = {}
                for color, integration in colors.items():
                    entry[int(color)] = integration

                calibrate_results[int(target)] = entry
            print("Loaded previous calibration parameters.")

        shift_value = 100
        horizontal_shift = 0
        vertical_shift = 0

        horizontal_quotient = float(top_right_x) / float(screen_width)
        horizontal_base_shift = horizontal_quotient * 3660 - 746
        # 1995
        vertical_quotient = float(top_right_y) / float(screen_height)
        vertical_base_shift = vertical_quotient * 2779
        # 2063

        write_reg = True
        while running:
            if write_reg:
                write_reg = False

                # painfully reverse-engineered numbers
                ArducamSDK.Py_ArduCam_writeSensorReg(handle, 0x0344, 112 + horizontal_base_shift + horizontal_shift)
                ArducamSDK.Py_ArduCam_writeSensorReg(handle, 0x0348, 112 + 1672 + horizontal_base_shift + horizontal_shift)
                ArducamSDK.Py_ArduCam_writeSensorReg(handle, 0x0346, 8 + 0 + vertical_base_shift + vertical_shift)
                ArducamSDK.Py_ArduCam_writeSensorReg(handle, 0x034A, 8 + 685 + vertical_base_shift + vertical_shift)
                ArducamSDK.Py_ArduCam_writeSensorReg(handle, 0x0202, coarse_integration)

            input_kb = str(sys.stdin.readline()).strip("\n")

            if input_kb == 'h' or input_kb == 'H':
                show_help()
            elif input_kb == 'q' or input_kb == 'Q':
                running = False
                print("Exiting...")
            elif input_kb == 'm' or input_kb == 'M':
                save_multiview = LED_MAX_ITERATIONS  # no output here
            elif input_kb == 't' or input_kb == 'T':
                save_single_flag = True  # no output here
            elif input_kb == 's' or input_kb == 'S':
                save_flag = True
                save_beginning = time.time()
                print("Recording started...")
            elif input_kb == 'c' or input_kb == 'C':
                save_flag = False
                print("Recording ended.")
            elif input_kb == 'l' or input_kb == 'L':  # move view to the left
                write_reg = True
                horizontal_shift = horizontal_shift - shift_value
            elif input_kb == 'r' or input_kb == 'R':  # move view to the right
                write_reg = True
                horizontal_shift = horizontal_shift + shift_value
            elif input_kb == 'u' or input_kb == 'U':  # move view up
                write_reg = True
                vertical_shift = vertical_shift - shift_value
            elif input_kb == 'd' or input_kb == 'D':  # move view down
                write_reg = True
                vertical_shift = vertical_shift + shift_value
            elif input_kb == 'w' or input_kb == 'W':  # increase coarse_integration (more white)
                write_reg = True
                coarse_integration = coarse_integration + 50
                print("Coarse_integration is now %d (0x%X)." % (coarse_integration, coarse_integration))
            elif input_kb == 'b' or input_kb == 'B':  # decrease coarse_integration (more black)
                write_reg = True
                coarse_integration = coarse_integration - 50
                print("Coarse_integration is now %d (0x%X)." % (coarse_integration, coarse_integration))
            elif input_kb == 'p' or input_kb == 'P':  # save the current parameters
                parameters = {
                    "mouse_x": mouse_x,
                    "mouse_y": mouse_y,
                    "draw_x": draw_x,
                    "draw_y": draw_y,
                    "coarse_integration": coarse_integration,
                    "calibration": calibrate_results
                }

                with open('recording_settings.json', 'w') as out:
                    json.dump(parameters, out, indent=4)

                print("Saved the current parameters.")
            elif input_kb == 'a' or input_kb == 'A':  # save the current parameters
                calibrate_flag = LED_MAX_ITERATIONS
                print("Recalibrating...")
            elif is_digit(input_kb):
                shift_value = int(input_kb)
                print("Changed shifting value to %d." % (shift_value))

        ct.join()
        rt.join()

        if ArducamSDK.Py_ArduCam_close(handle) == 0:
            print("Sucessfully closed device!")
        else:
            print("Failed to close device!")
