import sys
import os
import time
import cv2
import threading
import numpy as np
import signal
import json
from ImageConvert import *
import arducam_config_parser
import ArducamSDK
import math


sys.setrecursionlimit(10 ** 9)


global cfg, handle, running, Width, Heigth, save_flag, color_mode, save_raw
running = True
save_flag = False
save_raw = False
cfg = {}
handle = {}

screen_width = 1200
screen_height = screen_width / 4 * 3

resize_ratio = float(1200) / float(3664)
rectangle_width = float(916) * resize_ratio
rectangle_height = float(686) * resize_ratio

print("Screen: %d * %d" % (screen_width, screen_height))
print("Rectangle: %d * %d" % (rectangle_width, rectangle_height))

select = None
background_image = None
move_rectangle = False
mouse_x = 0
mouse_y = 0


def configBoard(config):
    global handle
    ArducamSDK.Py_ArduCam_setboardConfig(handle, config.params[0],
                                         config.params[1], config.params[2], config.params[3],
                                         config.params[4:config.params_length])


def camera_initFromFile(fileName, pWidth=None, pHeight=None):
    global cfg, handle, Width, Height, color_mode, save_raw
    config = arducam_config_parser.LoadConfigFile(fileName)

    camera_parameter = config.camera_param.getdict()

    if pWidth is None:
        Width = camera_parameter["WIDTH"]
    else:
        Width = pWidth

    if pHeight is None:
        Height = camera_parameter["HEIGHT"]
    else:
        Height = pHeight

    BitWidth = camera_parameter["BIT_WIDTH"]
    ByteLength = 1
    if BitWidth > 8 and BitWidth <= 16:
        ByteLength = 2
        save_raw = True
    FmtMode = camera_parameter["FORMAT"][0]
    color_mode = camera_parameter["FORMAT"][1]
    print("color mode", color_mode)

    I2CMode = camera_parameter["I2C_MODE"]
    I2cAddr = camera_parameter["I2C_ADDR"]
    TransLvl = camera_parameter["TRANS_LVL"]
    cfg = {
        "u32CameraType": 0x00,
        "u32Width": Width,
        "u32Height": Height,
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
        print("Serial: %c%c%c%c-%c%c%c%c-%c%c%c%c" % (datas[0], datas[1], datas[2], datas[3],
                                                      datas[4], datas[5], datas[6], datas[7],
                                                      datas[8], datas[9], datas[10], datas[11]))

        return True, handle
    else:
        print("open fail, rtn_val = ", ret)
        return False, handle


def captureImage_thread():
    global handle, running

    rtn_val = ArducamSDK.Py_ArduCam_beginCaptureImage(handle)
    if rtn_val != 0:
        print("Error beginning capture, rtn_val = ", rtn_val)
        running = False
        return
    else:
        print("Capture began, rtn_val = ", rtn_val)

    while running:
        # print "capture"
        rtn_val = ArducamSDK.Py_ArduCam_captureImage(handle)
        if rtn_val > 255:
            print("Error while capturing image, rtn_val = ", rtn_val)
            if rtn_val == ArducamSDK.USB_CAMERA_USB_TASK_ERROR:
                break
        time.sleep(0.005)

    running = False
    ArducamSDK.Py_ArduCam_endCaptureImage(handle)


def readImage_thread():
    global handle, running, Width, Height, save_flag, cfg, color_mode, save_raw
    count = 0
    totalFrame = 0
    time0 = time.time()
    time1 = time.time()
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
                print("read data fail!")
                continue

            image = convert_image(data, rtn_cfg, color_mode)

            time1 = time.time()
            if time1 - time0 >= 1:
                print("Frames per second: %d/s." % (count))
                count = 0
                time0 = time1
            count += 1
            if save_flag:
                cv2.imwrite("images/image%d.png" % totalFrame, image)
                if save_raw:
                    with open("images/image%d.raw" % totalFrame, 'wb') as f:
                        f.write(data)
                totalFrame += 1

            image = cv2.resize(image, (Width, Height), interpolation=cv2.INTER_LINEAR)

            cv2.imshow("ArduCam output", image)
            cv2.waitKey(10)
            ArducamSDK.Py_ArduCam_del(handle)
        else:
            time.sleep(0.001)


def showHelp():
    print(" usage: sudo python ArduCam_Py_Demo.py <path/config-file-name>	\
        \n\n example: sudo python ArduCam_Py_Demo.py ../../../python_config/AR0134_960p_Color.json	\
        \n\n While the program is running, you can press the following buttons in the terminal:	\
        \n\n 's' + Enter:Save the image to the images folder.	\
        \n\n 'c' + Enter:Stop saving images.	\
        \n\n 'q' + Enter:Stop running the program.	\
        \n\n")


def sigint_handler(signum, frame):
    global running, handle
    running = False
    exit()


def mouse(event, x, y, flags, params):
    global select, background_image, move_rectangle, mouse_x, mouse_y
    x = inborders(x, int(rectangle_width / 2), screen_width - int(rectangle_width / 2) - 1)
    y = inborders(y, int(rectangle_height / 2), screen_height - int(rectangle_height / 2) - 1)

    shifted_x = - x + screen_width
    shifted_y = y

    # print("center: %d; %d, top right: %d; %d, bottom left: %d; %d" % (shifted_x,
    #                                                                shifted_y,
    #                                                                shifted_x - rectangle_width / 2,
    #                                                                shifted_y - rectangle_height / 2,
    #                                                                shifted_x + rectangle_width / 2,
    #                                                                shifted_y + rectangle_height / 2))

    if event == cv2.EVENT_LBUTTONDOWN:
        move_rectangle = True

    elif event == cv2.EVENT_MOUSEMOVE:
        if move_rectangle:
            mouse_x = shifted_x
            mouse_y = shifted_y
            blank = background_image.copy()
            cv2.rectangle(blank,
                            (
                                x - int(rectangle_width / float(2)),
                                y - int(rectangle_height / float(2))
                            ),
                            (
                                x + int(rectangle_width / float(2)),
                                y + int(rectangle_height / float(2))
                            ),
                            (0, 0, 255),
                            1
                         )
            cv2.imshow(select, blank)

    elif event == cv2.EVENT_LBUTTONUP:
        move_rectangle = False
        mouse_x = shifted_x
        mouse_y = shifted_y
        blank = background_image.copy()
        cv2.rectangle(blank,
                        (
                            x - int(rectangle_width / float(2)),
                            y - int(rectangle_height / float(2))
                        ),
                        (
                            x + int(rectangle_width / float(2)),
                            y + int(rectangle_height / float(2))
                        ),
                        (0, 0, 255),
                        1
                     )
        cv2.imshow(select, blank)


def get_focus():
    global handle, running, Width, Height, cfg, color_mode, select, background_image

    while ArducamSDK.Py_ArduCam_availableImage(handle) == 0:
        time.sleep(0.001)

    rtn_val, data, rtn_cfg = ArducamSDK.Py_ArduCam_readImage(handle)
    datasize = rtn_cfg['u32Size']
    if rtn_val != 0 or datasize == 0:
        ArducamSDK.Py_ArduCam_del(handle)
        print("Failed to read data!")
        exit()

    background_image = convert_image(data, rtn_cfg, color_mode)
    background_image = cv2.resize(background_image, (screen_width, screen_height), interpolation=cv2.INTER_LINEAR)

    select = "Zone selection"
    cv2.namedWindow(select)
    cv2.setMouseCallback(select, mouse)

    cv2.imshow(select, background_image)

    while True:
        key = cv2.waitKey(0)

        if key == 13: # enter
            result = True
            break
        elif key == 27 & 0xFF: # escape
            result = False
            break

    cv2.destroyAllWindows()

    ArducamSDK.Py_ArduCam_del(handle)

    return result and mouse_x is not None and mouse_y is not None


signal.signal(signal.SIGINT, sigint_handler)
signal.signal(signal.SIGTERM, sigint_handler)

def is_digit(n):
    try:
        int(n)
        return True
    except ValueError:
        return False

def inborders(n, min_value, max_value):
    return max(min(n, max_value), min_value)

if __name__ == "__main__":
    showHelp()

    config_overview = "./config/3664_2748.cfg"
    config_focused = "./config/916_686.cfg"

    ret, handle = camera_initFromFile(config_overview)
    if ret:
        ArducamSDK.Py_ArduCam_setMode(handle, ArducamSDK.CONTINUOUS_MODE)

        ct = threading.Thread(target=captureImage_thread)
        ct.start()

        result = get_focus()

        running = False
        ct.join()

        ArducamSDK.Py_ArduCam_close(handle)

        if not result:
            exit()

        time.sleep(1)

        running = True

        ret, handle = camera_initFromFile(config_focused)

        ct = threading.Thread(target=captureImage_thread)
        rt = threading.Thread(target=readImage_thread)
        ct.start()
        rt.start()

        top_right_x = int(mouse_x - rectangle_width / 2)
        top_right_y = int(mouse_y - rectangle_height / 2)

        # print("mouse_x: %d" % (mouse_x))
        # print("mouse_y: %d" % (mouse_y))
        # print("top_right_x: %d" % (top_right_x))
        # print("top_right_y: %d" % (top_right_y))
        # print("rectangle_width: %d" % (rectangle_width))
        # print("rectangle_height: %d" % (rectangle_height))
        # print("screen_width: %d" % (screen_width))
        # print("screen_height: %d" % (screen_height))

        horizontal_quotient = float(top_right_x) / float(screen_width)
        horizontal_base_shift = horizontal_quotient * 3660 - 746
        # 1995
        vertical_quotient = float(top_right_y) / float(screen_height)
        vertical_base_shift = vertical_quotient * 2779
        # 2063

        # print("horizontal_quotient: %f" % (horizontal_quotient))
        # print("vertical_quotient: %f" % (vertical_quotient))

        ArducamSDK.Py_ArduCam_writeSensorReg(handle, 0x0206, 146 / 4) # analogue gain greenr
        ArducamSDK.Py_ArduCam_writeSensorReg(handle, 0x0208, 162 / 4) # analogue gain  red
        ArducamSDK.Py_ArduCam_writeSensorReg(handle, 0x020a, 168 / 4) # analogue gain  blue
        ArducamSDK.Py_ArduCam_writeSensorReg(handle, 0x020c, 144 / 4) # analogue gain  greenb

        ArducamSDK.Py_ArduCam_writeSensorReg(handle, 0x020E, 256) # digital gain greenr
        ArducamSDK.Py_ArduCam_writeSensorReg(handle, 0x0210, 256) # digital gain red
        ArducamSDK.Py_ArduCam_writeSensorReg(handle, 0x0212, 256) # digital gain blue
        ArducamSDK.Py_ArduCam_writeSensorReg(handle, 0x0214, 256) # digital gain greenb

        shift_value = 100
        horizontal_shift = 0
        vertical_shift = 0
        coarse_integration = 400
        write_reg = True
        while running:
            if write_reg:
                write_reg = False
                ArducamSDK.Py_ArduCam_writeSensorReg(handle, 0x0344, 112 + horizontal_base_shift + horizontal_shift)
                ArducamSDK.Py_ArduCam_writeSensorReg(handle, 0x0348, 112 + 1672 + horizontal_base_shift + horizontal_shift)
                ArducamSDK.Py_ArduCam_writeSensorReg(handle, 0x0346, 8 + 0 + vertical_base_shift + vertical_shift)
                ArducamSDK.Py_ArduCam_writeSensorReg(handle, 0x034A, 8 + 685 + vertical_base_shift + vertical_shift)
                ArducamSDK.Py_ArduCam_writeSensorReg(handle, 0x0202, coarse_integration)

            # print("horizontal_base_shift: %d" % (horizontal_base_shift))
            # print("vertical_base_shift: %d" % (vertical_base_shift))
            # print("horizontal_shift: %d" % (horizontal_shift))
            # print("vertical_shift: %d" % (vertical_shift))
            # print("final horizontal: %d to %d" % (112 + horizontal_shift, 112 + 1672 + horizontal_shift))
            # print("final vertical: %d to %d" % (8 + vertical_shift, 8 + 685 + vertical_shift))

            # min column start: -748
            # max column end:   3899 instead of 3871
            # horizontal size:  1672
            # min row start:    0
            # max row start:    2779 instead of 2755
            # vertical size:    685

            input_kb = str(sys.stdin.readline()).strip("\n")

            if input_kb == 'q' or input_kb == 'Q':
                running = False
            elif input_kb == 's' or input_kb == 'S':
                save_flag = True
            elif input_kb == 'c' or input_kb == 'C':
                save_flag = False
            elif input_kb == 'l' or input_kb == 'L': # move view to the left
                write_reg = True
                horizontal_shift = horizontal_shift - shift_value
            elif input_kb == 'r' or input_kb == 'R': # move view to the right
                write_reg = True
                horizontal_shift = horizontal_shift + shift_value
            elif input_kb == 'u' or input_kb == 'U': # move view up
                write_reg = True
                vertical_shift = vertical_shift - shift_value
            elif input_kb == 'd' or input_kb == 'D': # move view down
                write_reg = True
                vertical_shift = vertical_shift + shift_value
            elif input_kb == 'w' or input_kb == 'W': # increase coarse_integration (more white)
                write_reg = True
                coarse_integration = coarse_integration + 50
                print("Coarse_integration is now %d (0x%X)" % (coarse_integration, coarse_integration))
            elif input_kb == 'b' or input_kb == 'B': # decrease coarse_integration (more black)
                write_reg = True
                coarse_integration = coarse_integration - 50
                print("Coarse_integration is now %d (0x%X)" % (coarse_integration, coarse_integration))
            elif is_digit(input_kb):
                shift_value = int(input_kb)
                print("Changed shifting value to %d." % (shift_value))
        ct.join()
        rt.join()

        if ArducamSDK.Py_ArduCam_close(handle) == 0:
            print("Device close success!")
        else:
            print("Device close fail!")
