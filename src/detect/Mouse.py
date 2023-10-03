import cv2
import numpy as np

drawing = False
mode = True
ix, iy = -1, -1

class Mouse():
    def __init__(self):
	    self.img = np.zeros((200, 200, 3), np.uint8)

    def draw_circle(self, event, x, y, flags, param):
        global ix, iy, drawing, mode
        if event == cv2.EVENT_LBUTTONDOWN:
            drawing = True
            ix, iy = x, y
        elif event == cv2.EVENT_MOUSEMOVE and (flags & cv2.EVENT_FLAG_LBUTTON):
            if drawing == True:
                cv2.circle(self.img, (x, y), 3, (255, 255, 255), -1)
        elif event == cv2.EVENT_LBUTTONUP:
            drawing = False

    def draw_rectangle(self, event, x, y, flags, param):
        global image, begin, end

        image = self.img.copy()

        if event == cv2.EVENT_LBUTTONDOWN:
            begin = (x, y)

        elif event == cv2.EVENT_MOUSEMOVE and (flags & cv2.EVENT_FLAG_LBUTTON):
            cv2.rectangle(image, begin, (x, y), (0, 0, 255), 3)
            cv2.imshow('image', image)
    
        elif event == cv2.EVENT_LBUTTONUP:
            end = (x, y)
            cv2.rectangle(image, begin, end, (0,0,255), 3)
            cv2.imshow('image', image)

            min_x = min(begin[0], end[0])
            min_y = min(begin[1], end[1])
            width = abs(begin[0] - end[0])
            height = abs(begin[1] - end[1])

            cut_img = self.img[min_y:min_y+height, min_x:min_x+width]
            cv2.imshow('Cut', cut_img)
            cv2.imwrite('test.png', cut_img)

    def create_image(self):
        self.mode = True
        cv2.namedWindow('MNIST')

        while(1):
            cv2.imshow('MNIST', self.img)
			
            key = cv2.waitKey(1) & 0xFF
            if key == ord('c'):
                cv2.setMouseCallback('MNIST', self.draw_circle)
                self.mode = not mode
            elif key == ord('r'):
                cv2.setMouseCallback('MNIST', self.draw_rectangle)
            elif key == ord('q'):
                break
            elif key == 27:
                break

        cv2.destroyAllWindows()
		
if __name__ == '__main__':
    mn = Mouse()
    mn.create_image()
