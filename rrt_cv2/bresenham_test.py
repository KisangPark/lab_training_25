def bresenham_line(x1, y1, x2, y2): #return the list of points
    #get two points (integer values)
    #1) return longer axis
    #2) move pixel by pixel, check with the intermediate point
    points = []
    dx = abs(x2 - x1) #number of pixels
    dy = abs(y2 - y1)

    #adder -> determine the sign
    #easier think -> make order between x1 and x2
    sx = 1 if x1 < x2 else -1
    sy = 1 if y1 < y2 else -1
    err = dx - dy

    while True:
        points.append((x1, y1)) #x1 and y1 moving
        if x1 == x2 and y1 == y2:
            break

        #err2: two times difference of dx and dy
        err2 = err * 2
        if err2 > -dy:
            err -= dy
            x1 += sx
        if err2 < dx:
            err += dx
            y1 += sy

    return points

point_list = bresenham_line(0, 0, 2, 10)
print(point_list)