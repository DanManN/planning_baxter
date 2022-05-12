import cv2

print(cv2.__version__)
img = cv2.imread("start.png")
cv2.imshow("original", img)
cv2.waitKey(0)

cnts, _ = cv2.findContours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

cv2.drawContours(img, cnts, -1, [0, 0, 255], 2)

cv2.imshow("cnts", img)
cv2.waitKey(0)