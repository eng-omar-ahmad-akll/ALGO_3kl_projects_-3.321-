# by omar ahmed akll (3.321) -3KL-
# A* on a general graph - Tkinter visualiser
# Nodes are labeled with letters: a, b, c, ...
# UI features:
# - Draws an undirected weighted graph and places nodes on a circle
# - Click a node (while 'Mode' is Start/Goal) to set Start and Goal
# - Heuristic table: shows h(n) for each node; double-click a cell to edit
# - Distance table: shows g(n) = true cost from Start to that node after running A*
# - Run A* button: runs algorithm and highlights open/closed/path in the canvas
#
# Usage: save as graph_ast_gui.py and run: python graph_ast_gui.py

import tkinter as tk
from tkinter import ttk, simpledialog, messagebox
import math
import heapq

# ---------------- Sample graph definition ----------------
# adjacency: node -> list of (neighbor, weight)
SAMPLE_GRAPH = {
    'a': [('b', 75), ('c', 118),('e',140)],
    'b': [('a',75)],
    'c': [('a',118), ('d',111)],
    'd': [('c',111)],
    'e': [('a',140), ('g',80), ('f',99)],
    'f': [('e',99), ('i',211)],
    'g': [('e',80), ('h',97)],
    'h': [('g',97), ('i',101)],
    'i': [('h',101), ('f',211)]
}

# default heuristic values (these are arbitrary for demonstration)
SAMPLE_HEUR = {n: float(max(0, 10 - (ord(n)-ord('a')))) for n in SAMPLE_GRAPH}

# ---------------- Utility functions ----------------

def a_star(graph, start, goal, heur):
    '''Run A* on a general graph. Returns came_from, gscore, closed_set, open_set_order (for visualization)'''
    open_heap = []
    heapq.heappush(open_heap, (heur.get(start, 0), start))
    came_from = {}
    gscore = {start: 0.0}
    open_set = {start}
    closed = set()
    open_order = []  # sequence of popped nodes

    while open_heap:
        f, current = heapq.heappop(open_heap)
        open_set.discard(current)
        open_order.append(current)
        if current == goal:
            break
        closed.add(current)

        for neigh, w in graph.get(current, []):
            if neigh in closed:
                continue
            tentative = gscore[current] + w
            if tentative < gscore.get(neigh, float('inf')):
                came_from[neigh] = current
                gscore[neigh] = tentative
                fscore = tentative + heur.get(neigh, 0)
                if neigh not in open_set:
                    heapq.heappush(open_heap, (fscore, neigh))
                    open_set.add(neigh)
    return came_from, gscore, closed, open_order


def reconstruct_path(came_from, start, goal):
    if goal not in came_from and start != goal:
        # maybe start==goal
        if start==goal:
            return [start]
        return []
    path = [goal]
    cur = goal
    while cur != start:
        cur = came_from.get(cur)
        if cur is None:
            return []
        path.append(cur)
    path.reverse()
    return path

# ---------------- GUI ----------------

class GraphAStarApp(tk.Tk):
    def __init__(self, graph, heur):
        super().__init__()
        self.title('A* on Graph - Nodes as letters (a,b,c.. ) (3.321) say hi to 3KL')
        self.graph = graph
        self.nodes = sorted(graph.keys())
        self.heur = dict(heur)
        self.start = None
        self.goal = None
        self.node_pos = {}
        self.node_items = {}  # canvas id for circle
        self.text_items = {}  # canvas id for label
        self.node_radius = 22

        self._build_ui()
        self._place_nodes_circle()
        self._draw_edges()
        self._draw_nodes()
        self._populate_tables()

    def _build_ui(self):
        left = ttk.Frame(self)
        left.grid(row=0, column=0, sticky='ns')
        self.canvas = tk.Canvas(self, width=700, height=500, bg='white')
        self.canvas.grid(row=0, column=1, padx=8, pady=8)

        # Controls
        ttk.Label(left, text='Mode:').grid(row=0, column=0, pady=(6,0))
        self.mode = tk.StringVar(value='Start')
        ttk.Radiobutton(left, text='Start', variable=self.mode, value='Start').grid(row=1,column=0,sticky='w')
        ttk.Radiobutton(left, text='Goal', variable=self.mode, value='Goal').grid(row=2,column=0,sticky='w')
        ttk.Radiobutton(left, text='Inspect', variable=self.mode, value='Inspect').grid(row=3,column=0,sticky='w')

        ttk.Button(left, text='Run A*', command=self.run_astar).grid(row=4, column=0, pady=(10,0), sticky='we')
        ttk.Button(left, text='Clear Path', command=self.clear_path).grid(row=5, column=0, pady=(4,0), sticky='we')
        ttk.Button(left, text='Reset Heuristics', command=self.reset_heuristics).grid(row=6, column=0, pady=(6,0), sticky='we')

        ttk.Label(left, text='Heuristic (h) table: double-click value to edit').grid(row=7,column=0,pady=(8,0))
        self.heur_tree = ttk.Treeview(left, columns=('node','h'), show='headings', height=8)
        self.heur_tree.heading('node', text='Node')
        self.heur_tree.heading('h', text='Heuristic h(n)')
        self.heur_tree.column('node', width=40, anchor='center')
        self.heur_tree.column('h', width=80, anchor='center')
        self.heur_tree.grid(row=8,column=0)
        self.heur_tree.bind('<Double-1>', self.on_heur_double)

        ttk.Label(left, text='Distance (g) table: true cost from Start').grid(row=9,column=0,pady=(8,0))
        self.dist_tree = ttk.Treeview(left, columns=('node','g'), show='headings', height=8)
        self.dist_tree.heading('node', text='Node')
        self.dist_tree.heading('g', text='g(n)')
        self.dist_tree.column('node', width=40, anchor='center')
        self.dist_tree.column('g', width=80, anchor='center')
        self.dist_tree.grid(row=10,column=0, pady=(0,8))

        # Info label
        self.info_var = tk.StringVar(value='Click nodes on canvas to set Start/Goal depending on Mode')
        ttk.Label(left, textvariable=self.info_var, wraplength=180).grid(row=11, column=0)

        # Bind canvas click
        self.canvas.bind('<Button-1>', self.on_canvas_click)

    def _place_nodes_circle(self):
        # place nodes in a circle
        w = 700
        h = 500
        cx = w/2
        cy = h/2
        R = min(w,h)/2 - 80
        n = len(self.nodes)
        for i, node in enumerate(self.nodes):
            angle = 2*math.pi*i/n
            x = cx + R*math.cos(angle)
            y = cy + R*math.sin(angle)
            self.node_pos[node] = (x,y)

    def _draw_edges(self):
        # draw straight lines with weight labels
        for u in self.nodes:
            ux,uy = self.node_pos[u]
            for v,w in self.graph[u]:
                # draw each undirected edge once (u < v)
                if self.nodes.index(u) < self.nodes.index(v):
                    vx,vy = self.node_pos[v]
                    line = self.canvas.create_line(ux,uy,vx,vy, width=2)
                    # label mid-point
                    mx = (ux+vx)/2
                    my = (uy+vy)/2
                    self.canvas.create_text(mx, my-8, text=str(w), font=('Arial',9))

    def _draw_nodes(self):
        for node in self.nodes:
            x,y = self.node_pos[node]
            idc = self.canvas.create_oval(x-self.node_radius, y-self.node_radius,
                                            x+self.node_radius, y+self.node_radius,
                                            fill='#ffffff', outline='#000000', width=2)
            tid = self.canvas.create_text(x, y, text=node, font=('Arial',12,'bold'))
            self.node_items[node] = idc
            self.text_items[node] = tid

    def _populate_tables(self, gvals=None):
        # heuristics
        for i in self.heur_tree.get_children():
            self.heur_tree.delete(i)
        for n in self.nodes:
            self.heur_tree.insert('', 'end', iid=n, values=(n, f"{self.heur.get(n,0):.2f}" ))
        # distances
        for i in self.dist_tree.get_children():
            self.dist_tree.delete(i)
        for n in self.nodes:
            val = '' if gvals is None or n not in gvals else f"{gvals[n]:.2f}"
            self.dist_tree.insert('', 'end', iid='g_'+n, values=(n, val))

    def on_canvas_click(self, event):
        # find clicked node
        x = event.x
        y = event.y
        clicked = None
        for n, (nx,ny) in self.node_pos.items():
            if (x-nx)**2 + (y-ny)**2 <= (self.node_radius+2)**2:
                clicked = n
                break
        if not clicked:
            return
        mode = self.mode.get()
        if mode == 'Start':
            self.set_start(clicked)
        elif mode == 'Goal':
            self.set_goal(clicked)
        elif mode == 'Inspect':
            self.inspect_node(clicked)

    def set_start(self, node):
        if self.start:
            # reset color of old
            self.canvas.itemconfig(self.node_items[self.start], fill='#ffffff')
        self.start = node
        self.canvas.itemconfig(self.node_items[node], fill='#a7f3a7')
        self.info_var.set(f'Start set to {node}')
        # reset distances table
        self._populate_tables()

    def set_goal(self, node):
        if self.goal:
            self.canvas.itemconfig(self.node_items[self.goal], outline='#000000', width=2)
        self.goal = node
        self.canvas.itemconfig(self.node_items[node], outline='#ff0000', width=3)
        self.info_var.set(f'Goal set to {node}')

    def inspect_node(self, node):
        h = self.heur.get(node, 0)
        nb = self.graph.get(node, [])
        nb_str = ', '.join(f"{v}({w})" for v,w in nb)
        messagebox.showinfo('Node info', f'Node {node}\nheuristic h={h}\nneighbors: {nb_str}')

    def on_heur_double(self, event):
        item = self.heur_tree.identify_row(event.y)
        if not item:
            return
        col = self.heur_tree.identify_column(event.x)
        if col != '#2':
            return
        node = item
        cur = self.heur.get(node, 0)
        # prompt for new value
        s = simpledialog.askstring('Edit heuristic', f'Enter new heuristic for {node}:', initialvalue=f"{cur}")
        if s is None:
            return
        try:
            v = float(s)
        except:
            messagebox.showerror('Invalid', 'Please enter a number')
            return
        self.heur[node] = v
        self._populate_tables()

    def reset_heuristics(self):
        # reset to sample
        for i,n in enumerate(self.nodes):
            self.heur[n] = float(max(0, 10 - (ord(n)-ord('a'))))
        self._populate_tables()
        self.info_var.set('Heuristics reset to default sample values')

    def clear_path(self):
        # reset visual colors (except start/goal)
        for n in self.nodes:
            col = '#ffffff'
            if n == self.start:
                col = '#a7f3a7'
            self.canvas.itemconfig(self.node_items[n], fill=col, outline='#000000', width=2)
        # also remove any extra line items (edges were drawn at init; but path lines we draw with tag)
        self.canvas.delete('path_line')
        self.canvas.delete('open_line')
        self.info_var.set('Cleared path highlights')
        # reset distances table values
        self._populate_tables()

    def run_astar(self):
        if not self.start or not self.goal:
            messagebox.showwarning('Missing', 'Please set Start and Goal nodes first')
            return
        self.clear_path()
        came, gscore, closed, open_order = a_star(self.graph, self.start, self.goal, self.heur)
        path = reconstruct_path(came, self.start, self.goal)
        # update distances table with gscore (nodes not reachable remain blank or inf)
        gvals = {n: gscore.get(n, float('inf')) for n in self.nodes}
        # convert inf to empty string for display
        gdisp = {n: (gvals[n] if gvals[n] != float('inf') else None) for n in gvals}
        self._populate_tables(gdisp)

        # visualize: color closed nodes, color open popped order, draw path
        for n in closed:
            if n != self.start and n != self.goal:
                self.canvas.itemconfig(self.node_items[n], fill='#cfc9ff')
        # visualize open_order (in order popped)
        for i, n in enumerate(open_order):
            if n not in closed and n != self.start and n != self.goal:
                # draw small ring
                x,y = self.node_pos[n]
                r = self.node_radius+4
                self.canvas.create_oval(x-r, y-r, x+r, y+r, outline='#87CEFA', width=2, tags='open_line')
        # draw path
        if path:
            for i in range(len(path)-1):
                u = path[i]
                v = path[i+1]
                ux,uy = self.node_pos[u]
                vx,vy = self.node_pos[v]
                self.canvas.create_line(ux,uy,vx,vy, width=4, arrow='last', tags='path_line')
            # color path nodes
            for n in path:
                if n != self.start and n != self.goal:
                    self.canvas.itemconfig(self.node_items[n], fill='#FFD700')
            self.info_var.set(f'Path found: {"->".join(path)}    cost={gscore.get(self.goal,0):.2f}')
        else:
            self.info_var.set('No path found')


if __name__ == '__main__':
    app = GraphAStarApp(SAMPLE_GRAPH, SAMPLE_HEUR)
    app.mainloop()

    # here we finish give me your feedback +201014947877
    #(3.321) say thanks to 3KL 
    # hope you understand my project 
