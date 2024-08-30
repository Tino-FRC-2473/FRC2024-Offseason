import cv2

cap = cv2.VideoCapture(1)

while True:
    
    ret, frame = cap.read()
    cv2.imshow('result', frame)
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break
        
cap.release()
cv2.destroyAllWindows()
        