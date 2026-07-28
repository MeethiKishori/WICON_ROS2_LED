import sys
import cv2

src = sys.argv[1]
dst = src.replace('.pgm', '.png')

img = cv2.imread(src, cv2.IMREAD_GRAYSCALE)
if img is None:
    sys.exit(f'cannot read {src}')

cv2.imwrite(dst, img)
print(dst)
