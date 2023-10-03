import cv2 as cv
import numpy as np
import torch
from torchvision import transforms
from lenet import LeNet5
from PIL import Image, ImageTk
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String


device = torch.device('cuda')
transform = transforms.Compose([
    transforms.Resize([32, 32]),
    transforms.ToTensor()
])


def predict(curr_frame):
    net = LeNet5()
    net.load_state_dict(torch.load('12_best.pkl'))
    net.to(device)
    torch.no_grad()
    trans_frame = transform(curr_frame).unsqueeze(0)
    input_frame = trans_frame.to(device)
    outputs = net(input_frame)
    # _, predicted = torch.max(outputs, 1)
    # return predicted
    return outputs


thes = 180


def detect_circle():

    img = cap.read()[1]
# Convert to grayscale
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    global thes
    ret, binary = cv.threshold(gray, thes, 255, cv.THRESH_BINARY)

    kernel = cv.getStructuringElement(cv.MORPH_RECT, (3, 3))
    Xbinary = cv.morphologyEx(binary, cv.MORPH_OPEN, kernel, iterations=4)
    edges = cv.Canny(Xbinary, 50, 150, apertureSize=3)
    edges = cv.dilate(edges, kernel, iterations=1)

    circles = cv.HoughCircles(
        edges, cv.HOUGH_GRADIENT, 1, 100, param1=100, param2=40, minRadius=0, maxRadius=0)
    # 画圆

    if circles is not None:
        circles = circles.astype(np.int32)
        cx = 0
        cy = 0
        cr = 0
        for i in circles[0, :]:
            if i[2] > 40:
                cx = i[0]
                cy = i[1]
                cr = i[2]
        # cv.circle(img, (cx, cy), cr, (0, 255, 0), 2)
        # cv.imshow("img", img)
        # cv.waitKey(10)
        if cr == 0:
            return -1, -1, -1

        x = cx - cr*0.65
        y = cy - cr*0.65
        w = cr * 2*0.65
        h = cr * 2*0.65

        x = int(x)
        y = int(y)
        w = int(w)
        h = int(h)
        crop_img = binary[y:y+h, x:x+w]

        if crop_img.size == 0:
            return -1, cx, cy

        crop_img = 255 - crop_img
        # cv.imshow("crop_img", crop_img)
        # return 0, cx, cy
        # crop_img 外接矩形，findContours
        contours, hierarchy = cv.findContours(
            crop_img, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        # 找到最大的contour
        if len(contours) == 0:  # 无数字
            return 10, cx, cy
        max_contour = contours[0]
        for contour in contours:
            if cv.contourArea(contour) > cv.contourArea(max_contour):
                max_contour = contour
        # 找到最大contour的外接矩形
        x, y, w, h = cv.boundingRect(max_contour)

        crop_img = crop_img[y:y+h, x:x+w]

        # 将crop_img 填充成正方形
        if w > h:
            crop_img = cv.copyMakeBorder(crop_img, int(
                (w-h)/2), int((w-h)/2), 0, 0, cv.BORDER_CONSTANT, value=0)
        else:
            crop_img = cv.copyMakeBorder(crop_img, 0, 0, int(
                (h-w)/2), int((h-w)/2), cv.BORDER_CONSTANT, value=0)

        size_scale = 0.4
        if w > h:
            h = w

        crop_img = cv.copyMakeBorder(crop_img, int(
            h*size_scale), int(h*size_scale), int(h*size_scale), int(h*size_scale), cv.BORDER_CONSTANT, value=0)

        # cv.imshow("final", crop_img)
        # cv.waitKey(10)
        # crop_img
        # cv.imshow("crop", crop_img)
        # cv.waitKey(10)
        # return 10, cx, cy
        # crop_img = 255 - crop_img
        if crop_img.size == 0:
            return -1, cx, cy
        # cv.imshow("img", img)

        tot = np.zeros(shape=(11), dtype=np.int32)
        crop_img = cv.resize(crop_img, (32, 32))

        for angle in range(0, 360, 60):
            M = cv.getRotationMatrix2D((16, 16), angle, 1)
            rotate_img = cv.warpAffine(crop_img, M, (32, 32))
            # cv.imshow("rotate", rotate_img)
            # cv.imwrite("rotate.jpg", rotate_img)
            # exit()
            rotate_img = Image.fromarray(rotate_img)
            result = predict(rotate_img)
            for i in range(10):
                tot[i] += result[0][i].item()

        mx = 0
        for i in range(10):
            if tot[i] > tot[mx]:
                mx = i
        print(mx)
        return mx, circles[0][0][0], cy
    else:
        return -1, -1, -1


if __name__ == "__main__":
    detect_qr = 0
    # open camera
    cap = cv.VideoCapture(2)
    # set camera resolution
    cap.set(cv.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv.CAP_PROP_FRAME_HEIGHT, 480)

    rospy.init_node('detect_circle', anonymous=True)
    center_pub = rospy.Publisher('/target', PoseStamped, queue_size=10)
    qr_pub = rospy.Publisher('/qr', String, queue_size=10)
    qrcoder = cv.QRCodeDetector()

    # rot_img = cv.imread("rotate.jpg", 0)
    # rot_img = Image.fromarray(rot_img)
    # rt = predict(rot_img)
    # print(rt)
    # # rospy.info("start detect circle")
    # print("DETECT START!")
    rospy.Rate(10)

    while 1:
        num, x, y = detect_circle()
        if num != -1:
            # print(num)
            center = PoseStamped()
            center.header.stamp = rospy.Time.now()
            center.pose.position.x = (240-y)*0.00206
            center.pose.position.y = (320-x)*0.00206
            center.pose.position.z = num
            center_pub.publish(center)

    while rospy.is_shutdown() is False:
        if detect_qr == 0:
            frame = cap.read()[1]
            frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
            data, bbox, rectifiedImage = qrcoder.detectAndDecode(frame)
            if len(data) > 0:
                detect_qr = 1
                # print(data)
                qr_pub.publish(data)
        else:
            qr_pub.publish(data)
            num, x, y = detect_circle()
            if num != -1:
                # print(num)
                center = PoseStamped()
                center.header.stamp = rospy.Time.now()
                center.pose.position.x = (240-y)*0.00206
                center.pose.position.y = (320-x)*0.00206
                center.pose.position.z = num
                center_pub.publish(center)

        # rospy.sleep(0.1)
                # exit()
        # img = cap.read()[1]
        # cv.imshow("edges", frame)
        # cv.waitKey(10)

        # exit()
