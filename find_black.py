import sensor, image, time, math
from pyb import UART, LED

# Initialize UART3 with a baud rate of 115200
uart = UART(3, 115200)
LEDB = LED(3)  # Blue LED

# Color tracking thresholds (L Min, L Max, A Min, A Max, B Min, B Max)
black_thresholds = [(0, 15, -128, 127, -128, 127)]  # Black color thresholds
red_thresholds = [(32, 75, 26, 62, 0, 37)]  # Red color thresholds
blue_thresholds = [(32, 89, -48, -5, -44, -9)]  # Blue color thresholds

sensor.reset()  # Initialize the camera sensor
sensor.set_pixformat(sensor.RGB565)  # Set image color format to RGB565
sensor.set_framesize(sensor.QVGA)  # Set image resolution to QVGA
sensor.skip_frames(time=2000)  # Wait for the camera to stabilize
sensor.set_auto_gain(False)  # Disable auto gain
sensor.set_auto_whitebal(False)  # Disable white balance
clock = time.clock()

count = 'w'  # Initialize count variable to 'w' (no red object detected)
position = 'u'  # Initialize position variable to 'u' (middle position)

while True:
    clock.tick()
    img = sensor.snapshot()
    red_detected = False  # Initialize red detection flag
    blue_detected = False  # Initialize blue detection flag
    black_detected = False  # Initialize black detection flag

    # Detect red blobs
    for blob in img.find_blobs(red_thresholds, pixels_threshold=100, area_threshold=100, merge=True):
        red_detected = True
        # Draw detected red blobs
        if blob.elongation() > 0.5:
            img.draw_edges(blob.min_corners(), color=(255, 0, 0))
            img.draw_line(blob.major_axis_line(), color=(0, 255, 0))
            img.draw_line(blob.minor_axis_line(), color=(0, 0, 255))
            img.draw_rectangle(blob.rect())
            img.draw_cross(blob.cx(), blob.cy())
            img.draw_keypoints([(blob.cx(), blob.cy(), int(math.degrees(blob.rotation())))], size=20)

    # Detect blue blobs
    for blob in img.find_blobs(blue_thresholds, pixels_threshold=100, area_threshold=100, merge=True):
        blue_detected = True
        # Determine the position of the blue object in the image
        if blob.cx() < img.width() // 5:
            position = "L"  # Far left in the image
        elif img.width() // 5 < blob.cx() < img.width() * 2 // 5:
            position = "l"  # Left in the image
        elif img.width() * 2 // 5 < blob.cx() < img.width() * 3 // 5:
            position = "m"  # Middle in the image
        elif img.width() * 3 // 5 < blob.cx() < img.width() * 4 // 5:
            position = "r"  # Right in the image
        elif blob.cx() > img.width() * 4 // 5:
            position = "R"  # Far right in the image
        # Draw detected blue blobs
        if blob.elongation() > 0.5:
            img.draw_edges(blob.min_corners(), color=(0, 0, 255))
            img.draw_line(blob.major_axis_line(), color=(0, 255, 0))
            img.draw_line(blob.minor_axis_line(), color=(255, 0, 0))
            img.draw_rectangle(blob.rect())
            img.draw_cross(blob.cx(), blob.cy())
            img.draw_keypoints([(blob.cx(), blob.cy(), int(math.degrees(blob.rotation())))], size=20)

    # Detect black blobs
    for blob in img.find_blobs(black_thresholds, pixels_threshold=100, area_threshold=100, merge=True):
        black_detected = True
        if blob.elongation() > 0.5:
            if abs(blob.rotation()) < math.pi / 4 or abs(blob.rotation()) > 3 * math.pi / 4:
                count = 'x'  # Horizontal black line
            else:
                # Determine the position of the vertical black line in the image
                if blob.cx() < img.width() // 3:
                    count = 'Y'  # Left in the image
                elif img.width() // 3 < blob.cx() < img.width() * 2 // 3:
                    count = 's'  # Middle in the image
                else:
                    count = 'y'  # Right in the image
            img.draw_edges(blob.min_corners(), color=(255, 0, 0))
            img.draw_line(blob.major_axis_line(), color=(0, 255, 0))
            img.draw_line(blob.minor_axis_line(), color=(0, 0, 255))
            img.draw_rectangle(blob.rect())
            img.draw_cross(blob.cx(), blob.cy())
            img.draw_keypoints([(blob.cx(), blob.cy(), int(math.degrees(blob.rotation())))], size=20)

    if red_detected:
        position = 'N'  # Red object detected
    elif blue_detected:
        position = position  # Blue object detected, no red object detected
    elif black_detected:
        position = count  # Black object detected, no red or blue object detected
    else:
        position = 'n'  # No red, blue, or black object detected

    # Send position information via UART
    uart.write("%c" % (position))

    print("count:", count)
    print("position:", position)

    time.sleep_ms(50)  # Delay for 0.1 seconds


