import cv2 as cv

move_rectangle = False
BLUE = [255, 0, 0]

background = cv.imread('beach.jpg')
width = 20
height = 20


def mouse(event, x, y, flags, params):
    global BLUE, background

    if event == cv.EVENT_LBUTTONUP:
        blank = background.copy()
        cv.rectangle(blank, (x - int(0.5 * width), y - int(0.5 * height)), (x + int(0.5 * width), y + int(0.5 * height)), BLUE, -1)
        cv.imshow('draw', blank)

if __name__ == '__main__':
    cv.namedWindow('draw')
    cv.setMouseCallback('draw', mouse)

    cv.imshow('draw', background)

    while True:
        k = cv.waitKey(1)

        # waiting for esc to exit
        if k == 27 & 0xFF:
            break

    cv.destroyAllWindows()
