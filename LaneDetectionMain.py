import cv2
import numpy as np

working_scale = 0.30 # percentage
output_scale = 0.50 # percentage
camera = cv2.VideoCapture("Lane Detection Test Video-01.mp4")
output_y = int(camera.get(cv2.CAP_PROP_FRAME_HEIGHT) * output_scale)
output_x = int(camera.get(cv2.CAP_PROP_FRAME_WIDTH) * output_scale)
working_y = int(camera.get(cv2.CAP_PROP_FRAME_HEIGHT) * working_scale)
working_x = int(camera.get(cv2.CAP_PROP_FRAME_WIDTH) * working_scale)
half_working_x = working_x // 2#is uzed enough to have is own variable

#Parameters

# trapeze 
# all in percentage
trapeze_upper_base = 0.27
trapeze_lower_base_right_point = 0.20 # pozition of the point relative with x=working_x
trapeze_lower_base_left_point = 0.20  # pozition of the point relative with x=0
trapeze_upper_base_height = 0.23  # pozition of the base relative with y=working_y
trapeze_lower_base_height = 0.00  # pozition of the base relative with y=working_y

# blur
# recomanded odd number 
blur = 9  # 7 or 9 works good

# Sobel filter
sobel_filter_percentage = 1  # percentage
sobel_filter_threshold = 60   

# black borders
# all in percentage
black_border_sides = 0.1  # pozition of the border relative with 0 and working_x
black_border_down = 0.03  # pozition of the border relative with working_y
black_border_mid = 0.03  #  pozition of the border relative with half_working_x

#drawing lines
line_thicknes: int = 20
line_border_fragmentation: int = 20
line_border_limit = -0.10  # pozition of the lines heads relative with working_x

# Color code
color_good = (50, 250, 50) # green. no interfernace
color_out_of_border = (50, 50, 250)  # red
color_jump = (250, 50, 50)  # blue. when noice appears or suddenly changes

######################################################################################

line_limit_right: int= half_working_x * (1 - line_border_limit)
line_limit_left: int = half_working_x * line_border_limit

sobel_filter_vertical = np.float32([[-1, -2, -1], [0, 0, 0], [+1, +2, +1]])
sobel_filter_horizontally = np.transpose(sobel_filter_vertical)

frame_working_size_XY = (working_x, working_y)
frame_working_size_YX = (working_y, working_x)

# anti elicopter efect, stabilizeaza umpic linia
# creaza efect de tranzie intre 2 stari stabile
line_max_shift: int = half_working_x // line_border_fragmentation
line_shift = working_x * black_border_mid
line_shift = half_working_x + int(line_shift)

# trapeze stuf
trapeze_corner_upper_left = (int(working_x - working_x * ((1 - trapeze_upper_base) / 2)), int(working_y * (1 - trapeze_upper_base_height)))
trapeze_corner_upper_right = (int(working_x - working_x * ((1 - trapeze_upper_base) / 2 + trapeze_upper_base)), int(working_y * (1 - trapeze_upper_base_height)))
trapeze_corner_lower_left = (int(working_x * (1 - (trapeze_lower_base_left_point / 2))), int(working_y * (1 - trapeze_lower_base_height)))
trapeze_corner_lower_right = (int(working_x * (trapeze_lower_base_right_point / 2)), int(working_y * (1 - trapeze_lower_base_height)))

trapeze_corners_4_strech = np.float32([trapeze_corner_upper_right, trapeze_corner_lower_right, trapeze_corner_lower_left, trapeze_corner_upper_left])
frame_corners_4_strech = np.float32([(0, 0), (0, working_y), (working_x, working_y), (working_x, 0)])
stretch_frame_2_trapeze = cv2.getPerspectiveTransform(trapeze_corners_4_strech, frame_corners_4_strech)
stretch_trapeze_2_frame = cv2.getPerspectiveTransform(frame_corners_4_strech, trapeze_corners_4_strech)

trapeze = [trapeze_corner_upper_right, trapeze_corner_upper_left, trapeze_corner_lower_left, trapeze_corner_lower_right]
trapeze_corners = np.array(trapeze)

# initialization
line_left_bot = (0, 0)
line_left_top = (0, 0)
line_right_bot = (0, 0)
line_right_top = (0, 0)
line_left_shift = (0, 0)
line_right_shift = (0, 0)

line_color_left = (255, 255, 255)
line_color_right = (255, 255, 255)


def Linie_management(line_bot_x, line_top_x, line_last_bot_x, line_last_top_x, string_out_of_bound_message):
    # resolving out of bounds
    if line_last_bot_x == 0:
        if line_bot_x < line_limit_left:

            if line_top_x < line_limit_left:
                return line_limit_left, line_limit_left, color_out_of_border
            
            if line_top_x > line_limit_right:
                return line_limit_left, line_limit_right, color_out_of_border
            
        if line_bot_x < line_limit_right:

            if line_top_x < line_limit_left:
                return line_limit_right, line_limit_left, color_out_of_border
            
            if line_top_x > line_limit_right:
                return line_limit_right, line_limit_right, color_out_of_border
            
        return line_bot_x, line_top_x, color_good
    
    if line_bot_x < line_limit_left or line_top_x < line_limit_left or line_bot_x > line_limit_right or line_top_x > line_limit_right:
        print("Out of bound! keep last " + string_out_of_bound_message)
        return line_last_bot_x, line_last_top_x, color_out_of_border
        
    color_cod = color_good
    line_min_bot_x = (line_last_bot_x - line_max_shift)
    line_max_bot_x = (line_last_bot_x + line_max_shift)
    line_min_top_x = (line_last_top_x - line_max_shift)
    line_max_top_x = (line_last_top_x + line_max_shift)

    line_next_bot_x = line_bot_x
    line_next_top_x = line_top_x

    if line_bot_x < line_min_bot_x:
        line_next_bot_x = line_min_bot_x
        color_cod = color_jump
    if line_bot_x > line_max_bot_x:
        line_next_bot_x = line_max_bot_x
        color_cod = color_jump
    if line_top_x < line_min_top_x:
        line_next_top_x = line_min_top_x
        color_cod = color_jump
    if line_top_x > line_max_top_x:
        line_next_top_x = line_max_top_x
        color_cod = color_jump

    return line_next_bot_x, line_next_top_x, color_cod

while True:
    ret, frame = camera.read()
    if ret is False:
        break
    frame = cv2.resize(frame, frame_working_size_XY)

    frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    frame_trapeze_mask = np.zeros(frame_working_size_YX, dtype=np.uint8)
    frame_trapeze_mask = cv2.fillConvexPoly(frame_trapeze_mask, trapeze_corners, 255)

    frame_trapez = frame_trapeze_mask * frame_gray * 255

    frame_stretch_trapeze = cv2.warpPerspective(frame_trapez, stretch_frame_2_trapeze, frame_working_size_XY)

    frame_blur = cv2.blur(frame_stretch_trapeze, (blur, blur), 0)
    frame_blur_float32 = np.float32(frame_blur)

    frame_sobel_vertical = cv2.filter2D(frame_blur_float32, -1, sobel_filter_vertical)
    frame_sobel_horizontaly = cv2.filter2D(frame_blur_float32, -1, sobel_filter_horizontally)

    # frame_sobel =frame_sobel_orizontal * frame_sobel_vertical / 2
    frame_sobel = np.sqrt(frame_sobel_horizontaly ** 2 + frame_sobel_vertical ** 2)
    frame_sobel = frame_sobel.astype(np.uint8)
    frame_sobel = cv2.resize(frame_sobel, frame_working_size_XY)

    _, frame_binar = cv2.threshold(frame_sobel, sobel_filter_threshold, 255, cv2.THRESH_BINARY)

    frame_binar_border = frame_binar.copy()
    frame_binar_border[:, :(int(working_x * black_border_sides))] = 0
    frame_binar_border[:, -(int(working_x * black_border_sides)):] = 0
    frame_binar_border[-(int(working_y * black_border_down)):, :] = 0

    white_pixel = np.argwhere(frame_binar_border[:, :int(half_working_x - working_x * black_border_mid)] > 1)
    white_pixel_left_y_raw = white_pixel[:, 0]
    white_pixel_left_x_raw = white_pixel[:, 1]

    white_pixel = np.argwhere(frame_binar_border[:, -int(half_working_x - working_x * black_border_mid):] > 1)
    white_pixel_right_y_raw = white_pixel[:, 0]
    white_pixel_right_x_raw = white_pixel[:, 1]

    frame_line = frame_binar_border.copy()

    if len(white_pixel_left_x_raw) > 0:
        line_left_b, line_left_a = np.polynomial.polynomial.polyfit(
            white_pixel_left_x_raw,
            white_pixel_left_y_raw,
            deg=1)

        # y=(a*x)+b
        # x=(y-b)/a
        line_left_x_heads = ([0, working_y] - line_left_b) // line_left_a

        line_left_bot_x, line_left_top_x, line_color_left = Linie_management(
            line_left_x_heads[0],
            line_left_x_heads[1],
            line_left_bot[0],                                            
            line_left_top[0],
            string_out_of_bound_message = "left!")

        line_left_bot = (int(line_left_bot_x), 0)
        line_left_top = (int(line_left_top_x), working_y)

        frame_line = cv2.line(
            frame_line,
            line_left_bot,
            line_left_top,
            255,
            10)

    if len(white_pixel_right_x_raw) > 0:
        line_right_b, line_right_a = np.polynomial.polynomial.polyfit(
            white_pixel_right_x_raw,
            white_pixel_right_y_raw,
            deg=1)

        # y=(a*x)+b
        # x=(y-b)/a
        line_right_x_heads = ([0, working_y] - line_right_b) // line_right_a

        line_right_bot_x, line_right_top_x, line_color_right = Linie_management(
            line_right_x_heads[0],
            line_right_x_heads[1],
            line_right_bot[0],
            line_right_top[0],
            string_out_of_bound_message = "right!")

        line_right_bot = (int(line_right_bot_x), 0)
        line_right_top = (int(line_right_top_x), working_y)
 
        line_left_shift = (int(line_right_bot_x) + line_shift, 0)
        line_right_shift = (int(line_right_top_x) + line_shift, working_y)

        frame_line = cv2.line(
            frame_line,
            line_left_shift,
            line_right_shift,
            255,
            10)

    frame_line_only_right = np.zeros(frame_working_size_YX, dtype=np.uint8)
    frame_line_only_right = cv2.cvtColor(frame_line_only_right, cv2.COLOR_GRAY2RGB)
    frame_line_only_right = cv2.line(
        frame_line_only_right,
        line_left_shift,
        line_right_shift,
        line_color_right,
        line_thicknes)
    frame_line_only_right = cv2.warpPerspective(
        frame_line_only_right,
        stretch_trapeze_2_frame,
        frame_working_size_XY)

    frame_lini_only_left = np.zeros(frame_working_size_YX, dtype=np.uint8)
    frame_lini_only_left = cv2.cvtColor(frame_lini_only_left, cv2.COLOR_GRAY2RGB)
    frame_lini_only_left = cv2.line(
        frame_lini_only_left,
        line_left_bot,
        line_left_top,
        line_color_left,
        line_thicknes)
    frame_lini_only_left = cv2.warpPerspective(
        frame_lini_only_left,
        stretch_trapeze_2_frame,
        frame_working_size_XY)

    frame_final = frame.copy()
    frame_lini_color = frame_lini_only_left * 255 + frame_line_only_right * 255
    frame_final = frame_final + frame_lini_color
    frame_final = cv2.resize(frame_final, (output_x, output_y))

    cv2.imshow('Input', frame)
    cv2.imshow('Gray', frame_gray)
    cv2.imshow('Trapeze mask', frame_trapeze_mask)
    cv2.imshow('Trapeze', frame_trapez)
    cv2.imshow('Bird view', frame_stretch_trapeze)
    cv2.imshow('Blur', frame_blur)
    cv2.imshow('Sobel', frame_sobel)
    cv2.imshow('Binary', frame_binar)
    cv2.imshow('Bordered', frame_binar_border)
    cv2.imshow('Lines', frame_line)
    cv2.imshow('Lines Color', frame_lini_color)
    cv2.imshow('Final', frame_final)
    if cv2.waitKey(15) & 0xFF == ord('q'):
        break

camera.release()
cv2.destroyAllWindows()
