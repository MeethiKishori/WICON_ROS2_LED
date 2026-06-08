canvas.tobytes() converts the NumPy array (640×640×3 uint8) into raw bytes — that's 640×640×3 = 1,228,800 bytes of pixel data sent every 20ms over the /led/frames_raw topic.

encoding = 'bgr8' — tells the subscriber it's 3 channels, 8 bits each, BGR order
step = width * 3 — bytes per row (640 × 3 = 1920)
data = canvas.tobytes() — the actual pixels

step tells the subscriber how many bytes are in one row of the image.

For a 640-wide BGR image: 640 pixels × 3 bytes per pixel = 1920 bytes per row

It's needed because image data is stored as a flat 1D byte array. Without step, the subscriber doesn't know where one row ends and the next begins. It uses step to index into the data correctly:


row 0: data[0 : 1920]
row 1: data[1920 : 3840]
row 2: data[3840 : 5760]
...
