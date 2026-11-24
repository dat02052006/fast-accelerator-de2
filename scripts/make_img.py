import cv2
import sys

INPUT_FILE = "input.jpg"          # Đổi tên file ảnh của bạn ở đây (hoặc truyền qua tham số)
BIN_OUT    = "test_img.bin"       # File nhị phân nạp xuống kit DE2 qua Tcl
PNG_REF    = "img_640x480.png"    # File ảnh chuẩn dùng để chạy OpenCV đối chiếu

img = cv2.imread(INPUT_FILE, cv2.IMREAD_GRAYSCALE)
if img is None:
    print(f"[ERROR] Khong doc duoc file '{INPUT_FILE}'. Kiem tra lai ten file va duong dan!")
    sys.exit(1)

# Chuyển kích thước về đúng 640 (rộng) x 480 (cao)
img_640x480 = cv2.resize(img, (640, 480), interpolation=cv2.INTER_AREA)

# 1. Xuất file nhị phân thô (307,200 bytes) cho System Console
img_640x480.tofile(BIN_OUT)

# 2. Xuất file PNG không nén để chạy OpenCV kiểm chứng bit-exact
cv2.imwrite(PNG_REF, img_640x480)

print(f"[OK] Kich thuoc goc: {img.shape[1]}x{img.shape[0]} -> Da chuyen ve: 640x480")
print(f"[OK] Da tao '{BIN_OUT}' ({img_640x480.size} bytes) va '{PNG_REF}'")