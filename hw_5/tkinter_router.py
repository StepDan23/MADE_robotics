import math

import heapq
from tkinter import *
from shapely.geometry import Polygon

STEP_SIZE = 20
'''================= Your classes and methods ================='''


# These functions will help you to check collisions with obstacles

def rotate(points, angle, center):
    angle = math.radians(angle)
    cos_val = math.cos(angle)
    sin_val = math.sin(angle)
    cx, cy = center
    new_points = []

    for x_old, y_old in points:
        x_old -= cx
        y_old -= cy
        x_new = x_old * cos_val - y_old * sin_val
        y_new = x_old * sin_val + y_old * cos_val
        new_points.append((x_new + cx, y_new + cy))

    return new_points


def get_polygon_from_position(position):
    x, y, yaw = position
    points = [(x - 50, y - 100), (x + 50, y - 100), (x + 50, y + 100), (x - 50, y + 100), (x - 50, y - 100)]
    new_points = rotate(points, yaw * 180 / math.pi, (x, y))
    return Polygon(new_points)


def get_polygon_from_obstacle(obstacle):
    points = [(obstacle[0], obstacle[1]), (obstacle[2], obstacle[3]), (obstacle[4], obstacle[5]),
              (obstacle[6], obstacle[7]), (obstacle[0], obstacle[1])]
    return Polygon(points)


def collides(position, obstacle):
    return get_polygon_from_position(position).intersection(get_polygon_from_obstacle(obstacle))


class Node:
    def __init__(self, position, g, heuristic_fabric, children=None, root=None):
        self.pos = (int(position[0]), int(position[1]), round(position[2], 1))
        self.root = root
        self.children = [] if children is None else children
        self.g = g + 1
        self.priority = heuristic_fabric(self, self.root)

    def __eq__(self, other):
        _POS_EPS = 10
        _ANGLE_EPS = math.pi / 8

        if (abs(self.pos[0] - other.pos[0]) < _POS_EPS
                and abs(self.pos[1] - other.pos[1]) < _POS_EPS
                and abs(self.pos[2] - other.pos[2]) < _ANGLE_EPS):
            print('equals', self.pos, other.pos)
            return True
        else:
            return False


class PriorityQueue:
    def __init__(self):
        self.heap = []
        self.count = 0

    def push(self, item, priority):
        entry = (priority, self.count, item)
        heapq.heappush(self.heap, entry)
        self.count += 1

    def update(self, item, priority):
        for index, (p, c, i) in enumerate(self.heap):
            if i == item:
                if p <= priority:
                    break
                del self.heap[index]
                self.heap.append((priority, c, item))
                heapq.heapify(self.heap)
                break
        else:
            self.push(item, priority)

    def pop(self):
        (_, _, item) = heapq.heappop(self.heap)
        return item

    def is_empty(self):
        return len(self.heap) == 0


class Window:
    """================= Your Main Function ================="""

    def __init__(self):
        self.root = Tk()
        self.root.title("")
        self.width = self.root.winfo_screenwidth()
        self.height = self.root.winfo_screenheight()
        self.root.geometry(f'{self.width}x{self.height}')
        self.canvas = Canvas(self.root, bg="#777777", height=self.height, width=self.width)
        self.canvas.pack()
        self.obstacles = []
        self.target = None

    def go(self, event):

        # Write your code here

        print("Start position:", self.get_start_position())
        print("Target position:", self.get_target_position())
        self.obstacles = self.get_obstacles()
        print("Obstacles:", self.obstacles)

        # Example of collision calculation

        number_of_collisions = 0
        for obstacle in self.obstacles:
            if collides(self.get_start_position(), obstacle):
                number_of_collisions += 1
        print("Start position collides with", number_of_collisions, "obstacles")

        route = self.get_route()
        self.draw_final_route(route)

    def get_route(self):
        route_queue = PriorityQueue()
        start_node = Node(self.get_start_position(), 0, lambda x, y: 0)
        route_queue.push(start_node, start_node)
        self.target = Node(self.get_target_position(), 0, lambda x, y: 0)

        while not route_queue.is_empty():
            node = route_queue.pop()

            self.draw_node(node)
            self.canvas.update_idletasks()
            if self.target == node:
                return node

            self.make_step(node)
            for child in node.children:
                route_queue.update(child, child.priority)

    def make_step(self, node):
        _N_DIRECTION = 16

        for angle_id in range(_N_DIRECTION + 1):
            x, y, yaw = node.pos
            yaw = (yaw + angle_id * math.pi / _N_DIRECTION) % (2 * math.pi)
            x += math.cos(yaw) * STEP_SIZE
            y += math.sin(yaw) * STEP_SIZE
            pos = (x, y, yaw)
            if self.valid_move(pos):
                node.children.append(Node(pos, node.g, self.heuristic, root=node))

    def heuristic(self, cur_node, prev_node):
        x, y, yaw = cur_node.pos
        dist_obsc = 0
        for obstacle in self.obstacles:
            dist_obsc -= self.distance(x, y, obstacle[0], obstacle[1])
        dist = self.distance(x, y, self.target.pos[0], self.target.pos[1])
        prev_turn = abs(prev_node.pos[2] - cur_node.pos[2]) // (math.pi / 2)

        return cur_node.g + prev_turn + dist + dist_obsc / (10 * len(self.obstacles) + 1)

    def valid_move(self, position):
        return not any(collides(position, obstacle) for obstacle in self.obstacles)

    def draw_node(self, node, color='blue'):
        self.canvas.create_oval(node.pos[0] - 2.0, node.pos[1] - 2.0,
                                node.pos[0] + 2.0, node.pos[1] + 2.0,
                                outline=color, fill=color, tag='way_point')

    def draw_final_route(self, route: Node):
        while route.root is not None:
            x_0, y_0, _ = route.pos
            x_1, y_1, _ = route.root.pos
            self.canvas.create_line(x_0, y_0, x_1, y_1, width=3, tag='way_point')
            route = route.root
        print('route printed')

    '''================= Interface Methods ================='''

    def get_obstacles(self):
        obstacles = []
        potential_obstacles = self.canvas.find_all()
        for i in potential_obstacles:
            if (i > 2):
                coords = self.canvas.coords(i)
                if coords:
                    obstacles.append(coords)
        return obstacles

    def get_start_position(self):
        x, y = self.get_center(2)  # Purple block has id 2
        yaw = self.get_yaw(2)
        return x, y, yaw

    def get_target_position(self):
        x, y = self.get_center(1)  # Green block has id 1
        yaw = self.get_yaw(1)
        return x, y, yaw

    def get_center(self, id_block):
        coords = self.canvas.coords(id_block)
        center_x, center_y = ((coords[0] + coords[4]) / 2, (coords[1] + coords[5]) / 2)
        return [center_x, center_y]

    def get_yaw(self, id_block):
        center_x, center_y = self.get_center(id_block)
        first_x = 0.0
        first_y = -1.0
        second_x = 1.0
        second_y = 0.0
        points = self.canvas.coords(id_block)
        end_x = (points[0] + points[2]) / 2
        end_y = (points[1] + points[3]) / 2
        direction_x = end_x - center_x
        direction_y = end_y - center_y
        length = math.hypot(direction_x, direction_y)
        unit_x = direction_x / length
        unit_y = direction_y / length
        cos_yaw = unit_x * first_x + unit_y * first_y
        sign_yaw = unit_x * second_x + unit_y * second_y
        if (sign_yaw >= 0):
            return math.acos(cos_yaw)
        else:
            return -math.acos(cos_yaw)

    def get_vertices(self, id_block):
        return self.canvas.coords(id_block)

    '''=================================================='''

    def rotate(self, points, angle, center):
        angle = math.radians(angle)
        cos_val = math.cos(angle)
        sin_val = math.sin(angle)
        cx, cy = center
        new_points = []

        for x_old, y_old in points:
            x_old -= cx
            y_old -= cy
            x_new = x_old * cos_val - y_old * sin_val
            y_new = x_old * sin_val + y_old * cos_val
            new_points.append(x_new + cx)
            new_points.append(y_new + cy)

        return new_points

    def start_block(self, event):
        widget = event.widget
        widget.start_x = event.x
        widget.start_y = event.y

    def in_rect(self, point, rect):
        x_start, x_end = min(rect[::2]), max(rect[::2])
        y_start, y_end = min(rect[1::2]), max(rect[1::2])

        if x_start < point[0] < x_end and y_start < point[1] < y_end:
            return True

    def motion_block(self, event):
        widget = event.widget

        for i in range(1, 10):
            if widget.coords(i) == []:
                break
            if self.in_rect([event.x, event.y], widget.coords(i)):
                coords = widget.coords(i)
                id = i
                break

        res_cords = []
        try:
            coords
        except:
            return

        for ii, i in enumerate(coords):
            if ii % 2 == 0:
                res_cords.append(i + event.x - widget.start_x)
            else:
                res_cords.append(i + event.y - widget.start_y)

        widget.start_x = event.x
        widget.start_y = event.y
        widget.coords(id, res_cords)
        widget.center = ((res_cords[0] + res_cords[4]) / 2, (res_cords[1] + res_cords[5]) / 2)

    def draw_block(self, points, color):
        x = self.canvas.create_polygon(points, fill=color)
        return x

    def distance(self, x1, y1, x2, y2):
        return ((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5

    def set_id_block(self, event):
        widget = event.widget

        for i in range(1, 10):
            if widget.coords(i) == []:
                break
            if self.in_rect([event.x, event.y], widget.coords(i)):
                coords = widget.coords(i)
                id = i
                widget.id_block = i
                break

        widget.center = ((coords[0] + coords[4]) / 2, (coords[1] + coords[5]) / 2)

    def rotate_block(self, event):
        angle = 0
        widget = event.widget

        if widget.id_block == None:
            for i in range(1, 10):
                if widget.coords(i) == []:
                    break
                if self.in_rect([event.x, event.y], widget.coords(i)):
                    coords = widget.coords(i)
                    id = i
                    widget.id_block == i
                    break
        else:
            id = widget.id_block
            coords = widget.coords(id)

        wx, wy = event.x_root, event.y_root
        try:
            coords
        except:
            return

        block = coords
        center = widget.center
        x, y = block[2], block[3]

        cat1 = self.distance(x, y, block[4], block[5])
        cat2 = self.distance(wx, wy, block[4], block[5])
        hyp = self.distance(x, y, wx, wy)

        if wx - x > 0:
            angle = math.acos((cat1 ** 2 + cat2 ** 2 - hyp ** 2) / (2 * cat1 * cat2))
        elif wx - x < 0:
            angle = -math.acos((cat1 ** 2 + cat2 ** 2 - hyp ** 2) / (2 * cat1 * cat2))

        new_block = self.rotate([block[0:2], block[2:4], block[4:6], block[6:8]], angle, center)
        self.canvas.coords(id, new_block)

    def delete_block(self, event):
        widget = event.widget.children["!canvas"]

        for i in range(1, 10):
            if widget.coords(i) == []:
                break
            if self.in_rect([event.x, event.y], widget.coords(i)):
                widget.coords(i, [0, 0])
                break

    def create_block(self, event):
        block = [[0, 100], [100, 100], [100, 300], [0, 300]]

        id = self.draw_block(block, "black")

        self.canvas.tag_bind(id, "<Button-1>", self.start_block)
        self.canvas.tag_bind(id, "<Button-3>", self.set_id_block)
        self.canvas.tag_bind(id, "<B1-Motion>", self.motion_block)
        self.canvas.tag_bind(id, "<B3-Motion>", self.rotate_block)

    def make_draggable(self, widget):
        widget.bind("<Button-1>", self.drag_start)
        widget.bind("<B1-Motion>", self.drag_motion)

    def drag_start(self, event):
        widget = event.widget
        widget.start_x = event.x
        widget.start_y = event.y

    def drag_motion(self, event):
        widget = event.widget
        x = widget.winfo_x() - widget.start_x + event.x + 200
        y = widget.winfo_y() - widget.start_y + event.y + 100
        widget.place(rely=0.0, relx=0.0, x=x, y=y)

    def create_button_create(self):
        button = Button(
            text="New",
            bg="#555555",
            activebackground="blue",
            borderwidth=0
        )

        button.place(rely=0.0, relx=0.0, x=200, y=100, anchor=SE, width=200, height=100)
        button.bind("<Button-1>", self.create_block)

    def create_green_block(self, center_x):
        block = [[center_x - 50, 100],
                 [center_x + 50, 100],
                 [center_x + 50, 300],
                 [center_x - 50, 300]]

        id = self.draw_block(block, "green")

        self.canvas.tag_bind(id, "<Button-1>", self.start_block)
        self.canvas.tag_bind(id, "<Button-3>", self.set_id_block)
        self.canvas.tag_bind(id, "<B1-Motion>", self.motion_block)
        self.canvas.tag_bind(id, "<B3-Motion>", self.rotate_block)

    def create_purple_block(self, center_x, center_y):
        block = [[center_x - 50, center_y - 300],
                 [center_x + 50, center_y - 300],
                 [center_x + 50, center_y - 100],
                 [center_x - 50, center_y - 100]]

        id = self.draw_block(block, "purple")

        self.canvas.tag_bind(id, "<Button-1>", self.start_block)
        self.canvas.tag_bind(id, "<Button-3>", self.set_id_block)
        self.canvas.tag_bind(id, "<B1-Motion>", self.motion_block)
        self.canvas.tag_bind(id, "<B3-Motion>", self.rotate_block)

    def create_button_reset(self):
        button = Button(
            text="Reset",
            bg="#555555",
            activebackground="blue",
            borderwidth=0
        )

        button.place(rely=0.0, relx=1.0, x=-100, y=200, anchor=SE, width=100, height=200)
        button.bind("<Button-1>", self.del_all)

    def del_all(self, event):
        self.canvas.delete("way_point")

    def create_button_go(self):
        button = Button(
            text="Go",
            bg="#555555",
            activebackground="blue",
            borderwidth=0
        )

        button.place(rely=0.0, relx=1.0, x=0, y=200, anchor=SE, width=100, height=200)
        button.bind("<Button-1>", self.go)

    def run(self):
        root = self.root

        self.create_button_create()
        self.create_button_reset()
        self.create_button_go()
        self.create_green_block(self.width / 2)
        self.create_purple_block(self.width / 2, self.height)

        root.bind("<Delete>", self.delete_block)

        root.mainloop()


if __name__ == "__main__":
    run = Window()
    run.run()
