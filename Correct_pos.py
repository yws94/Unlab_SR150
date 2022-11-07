import math

'''TAG POSITION'''
############################
#                          #
#   1 ---------------- 2   #  TAG1 & TAG2
#   |  \ ang_b         |   #
#   |   \              |   #
#   |    \  [x]        |   #   x = Target Position
#   |     \            | d #  
#   |ang_a \           | i #
#   |       ^          | s # 
#   |     / | \        | t #
#   |       |          | - #  
#   |       |          | y #
#   |   dist_diag      |   #
#   |                  |   #
#   3 ---------------- 4   #  TAG3 & TAG4
#          dist_x          #
############################


dist_x = 0.45  # 12 21 34 43
dist_y = 0.6   # 13 24 31 42
dist_diag = round(math.sqrt(pow(dist_x,2)+pow(dist_y,2)),3)  # 14 23 32 41
m,n = 1, 2

pi = math.pi
ang_a = math.atan2(dist_x, dist_y)
ang_b = math.atan2(dist_y, dist_x)

def corr_pos(tag_pos):
    n = len(tag_pos)
    if 2 <= n <= 4:
        tag_pos.sort(key = lambda x : x[3])
        f_node, s_node = tag_pos[0], tag_pos[1]
        f_x, f_y, s_x, s_y = f_node[1], f_node[2], s_node[1], s_node[2]
        rad = math.atan2(s_y - f_y, s_x - f_x)

        if f_node[0] == '11':
            if s_node[0] == '22': # First Node = Tag1, Second Node = Tag2
                s_x, s_y = f_x + dist_x * math.cos(rad), f_y + dist_x * math.sin(rad)
                x3, y3 = f_x + dist_y * math.cos((rad + 1.5 * pi)%(2 * pi)), f_y + dist_y * math.sin((rad + 1.5 * pi)%(2 * pi))
                x4, y4 = s_x + dist_y * math.cos((rad + 1.5 * pi)%(2 * pi)), s_y + dist_y * math.sin((rad + 1.5 * pi)%(2 * pi))
                pos1, pos2, pos3, pos4 = [f_x, f_y], [round(s_x,3), round(s_y,3)], [round(x3,3), round(y3,3)], [round(x4,3), round(y4,3)]
                
            elif s_node[0] == '33': # First Node = Tag1, Second Node = Tag3
                s_x, s_y = f_x + dist_y * math.cos(rad), f_y + dist_y * math.sin(rad)
                x2, y2 = f_x + dist_x * math.cos((rad + 0.5 * pi)%(2 * pi)), f_y + dist_x * math.sin((rad + 0.5 * pi)%(2 * pi))
                x4, y4 = s_x + dist_x * math.cos((rad + 0.5 * pi)%(2 * pi)), s_y + dist_x * math.sin((rad + 0.5 * pi)%(2 * pi))
                pos1, pos2, pos3, pos4 = [f_x, f_y], [round(x2,3), round(y2,3)], [round(s_x,3), round(s_y,3)], [round(x4,3), round(y4,3)]
                
            elif s_node[0] == '44': # First Node = Tag1, Second Node = Tag4
                s_x, s_y = f_x + dist_diag * math.cos(rad), f_y + dist_diag * math.sin(rad)
                x3, y3 = f_x + dist_y * math.cos(rad - ang_a), f_y + dist_y * math.sin(rad - ang_a)
                x2, y2 = f_x + dist_x * math.cos(rad + ang_b), f_y + dist_x * math.sin(rad + ang_b)
                pos1, pos2, pos3, pos4 = [f_x, f_y], [round(x2,3), round(y2,3)], [round(x3,3), round(y3,3)], [round(s_x,3), round(s_y,3)]

                
        elif f_node[0] == '22':
            if s_node[0] == '11': # First Node = Tag2, Second Node = Tag1
                s_x, s_y = f_x + dist_x * math.cos(rad), f_y + dist_x * math.sin(rad)
                x4, y4 = f_x + dist_y * math.cos((rad + 0.5 * pi)%(2 * pi)), f_y + dist_y * math.sin((rad + 0.5 * pi)%(2 * pi))
                x3, y3 = s_x + dist_y * math.cos((rad + 0.5 * pi)%(2 * pi)), s_y + dist_y * math.sin((rad + 0.5 * pi)%(2 * pi))
                pos2, pos1, pos4, pos3 = [f_x, f_y], [round(s_x,3), round(s_y,3)], [round(x4,3), round(y4,3)], [round(x3,3), round(y3,3)]
                
            elif s_node[0] == '33': # First Node = Tag2, Second Node = Tag3
                s_x, s_y = f_x + dist_diag * math.cos(rad), f_y + dist_diag * math.sin(rad)
                x4, y4 = f_x + dist_y * math.cos(rad + ang_a), f_y + dist_y * math.sin(rad + ang_a)
                x1, y1 = f_x + dist_x * math.cos(rad - ang_b), f_y + dist_x * math.sin(rad - ang_b)
                pos2, pos3, pos4, pos1 = [f_x, f_y], [round(s_x,3), round(s_y,3)], [round(x4,3), round(y4,3)], [round(x1,3), round(y1,3)]   
                
            elif s_node[0] == '44': # First Node = Tag2, Second Node = Tag4
                s_x, s_y = f_x + dist_y * math.cos(rad), f_y + dist_y * math.sin(rad)
                x1, y1 = f_x + dist_x * math.cos((rad + 1.5 * pi)%(2 * pi)), f_y + dist_x * math.sin((rad + 1.5 * pi)%(2 * pi))
                x3, y3 = s_x + dist_x * math.cos((rad + 1.5 * pi)%(2 * pi)), s_y + dist_x * math.sin((rad + 1.5 * pi)%(2 * pi))
                pos2, pos4, pos1, pos3 = [f_x, f_y], [round(s_x,3), round(s_y,3)], [round(x1,3), round(y1,3)], [round(x3,3), round(y3,3)] 
        
        
        elif f_node[0] == '33':
            if s_node[0] == '11': # First Node = Tag3, Second Node = Tag1
                s_x, s_y = f_x + dist_y * math.cos(rad), f_y + dist_y * math.sin(rad)
                x4, y4 = f_x + dist_x * math.cos((rad + 1.5 * pi)%(2 * pi)), f_y + dist_x * math.sin((rad + 1.5 * pi)%(2 * pi))
                x2, y2 = s_x + dist_x * math.cos((rad + 1.5 * pi)%(2 * pi)), s_y + dist_x * math.sin((rad + 1.5 * pi)%(2 * pi))
                pos3, pos1, pos4, pos2 = [f_x, f_y], [round(s_x,3), round(s_y,3)], [round(x4,3), round(y4,3)], [round(x2,3), round(y2,3)]   
                
            elif s_node[0] == '22': # First Node = Tag3, Second Node = Tag2
                s_x, s_y = f_x + dist_diag * math.cos(rad), f_y + dist_diag * math.sin(rad)
                x1, y1 = f_x + dist_y * math.cos(rad + ang_a), f_y + dist_y * math.sin(rad + ang_a)
                x4, y4 = f_x + dist_x * math.cos(rad - ang_b), f_y + dist_x * math.sin(rad - ang_b)
                pos3, pos2, pos1, pos4 = [f_x, f_y], [round(s_x,3), round(s_y,3)], [round(x1,3), round(y1,3)], [round(x4,3), round(y4,3)]   
                
            elif s_node[0] == '44': # First Node = Tag3, Second Node = Tag4
                s_x, s_y = f_x + dist_x * math.cos(rad), f_y + dist_x * math.sin(rad)
                x1, y1 = f_x + dist_y * math.cos((rad + 0.5 * pi)%(2 * pi)), f_y + dist_y * math.sin((rad + 0.5 * pi)%(2 * pi))
                x2, y2 = s_x + dist_y * math.cos((rad + 0.5 * pi)%(2 * pi)), s_y + dist_y * math.sin((rad + 0.5 * pi)%(2 * pi))
                pos3, pos4, pos1, pos2 = [f_x, f_y], [round(s_x,3), round(s_y,3)], [round(x1,3), round(y1,3)], [round(x2,3), round(y2,3)]   

                
        elif f_node[0] == '44':
            if s_node[0] == '11': # First Node = Tag4, Second Node = Tag1
                s_x, s_y = f_x + dist_diag * math.cos(rad), f_y + dist_diag * math.sin(rad)
                x3, y3 = f_x + dist_x * math.cos(rad + ang_b), f_y + dist_x * math.sin(rad + ang_b)
                x2, y2 = f_x + dist_y * math.cos(rad - ang_a), f_y + dist_y * math.sin(rad - ang_a)
                pos4, pos1, pos3, pos2 = [f_x, f_y], [round(s_x,3), round(s_y,3)], [round(x3,3), round(y3,3)], [round(x2,3), round(y2,3)]   
                
            elif s_node[0] == '22': # First Node = Tag4, Second Node = Tag2
                s_x, s_y = f_x + dist_y * math.cos(rad), f_y + dist_y * math.sin(rad)
                x3, y3 = f_x + dist_x * math.cos((rad + 0.5 * pi)%(2 * pi)), f_y + dist_x * math.sin((rad + 0.5 * pi)%(2 * pi))
                x1, y1 = s_x + dist_x * math.cos((rad + 0.5 * pi)%(2 * pi)), s_y + dist_x * math.sin((rad + 0.5 * pi)%(2 * pi))
                pos4, pos2, pos3, pos1 = [f_x, f_y], [round(s_x,3), round(s_y,3)], [round(x3,3), round(y3,3)], [round(x1,3), round(y1,3)]  
                
            elif s_node[0] == '33': # First Node = Tag4, Second Node = Tag3
                s_x, s_y = f_x + dist_x * math.cos(rad), f_y + dist_x * math.sin(rad)
                x2, y2 = f_x + dist_y * math.cos((rad + 1.5 * pi)%(2 * pi)), f_y + dist_y * math.sin((rad + 1.5 * pi)%(2 * pi))
                x1, y1 = s_x + dist_y * math.cos((rad + 1.5 * pi)%(2 * pi)), s_y + dist_y * math.sin((rad + 1.5 * pi)%(2 * pi))
                pos4, pos3, pos2, pos1 = [f_x, f_y], [round(s_x,3), round(s_y,3)], [round(x2,3), round(y2,3)], [round(x1,3), round(y1,3)]   
        
        
        mid_12, mid_34 = [(pos1[0] + pos2[0])/2, (pos1[1] + pos2[1])/2], [(pos3[0] + pos4[0])/2, (pos3[1] + pos4[1])/2]
        target = [round((m * mid_34[0] + n * mid_12[0])/(m + n),3), round((m * mid_34[1] + n * mid_12[1])/(m + n),3)]
        
        print(f_node, s_node, [f_x, f_y], [round(s_x,3), round(s_y,3)])
        print(pos1, pos2, pos3, pos4, target)
        # return target
    else:
        pass
    
    

# print(((0.83333 * 3.14159)+1.5*pi)%(2*pi))
# print(((0.83333 * 3.14159)+1.5*pi)%(2*pi)/pi)