import numpy as np
import cv2, time

"""
rgb(66, 188, 244)
rgb(242, 138, 198)
"""
class Maze():

    class Node():
        def __init__(self, pos):
            self.pos = pos
            self.x = pos[0]
            self.y = pos[1]
            self.neighbours = [None, None, None, None]


    def __init__(self, imgloc):

        self.img = cv2.imread(imgloc, 0)
        self.height = self.img.shape[0] 
        self.width = self.img.shape[1]

        self.notFoundColorBGR = [244, 188, 64]
        self.foundPathColorBGR = [198, 138, 242]

        print("Maze initialized!")

    def createNodes(self):

        topnodes = {}

        n = None

        #Top node
        for i in range(self.width):
            if self.img[0][i] == 255:
                n = Maze.Node((0,i))
                self.start_node = n
                topnodes[i] = n

        for i in range(1, self.height-1):

            prv = False
            cur = self.img[i][0] == 255
            nxt = self.img[i][1] == 255

            leftnode = None

            for j in range(1, self.width-1):

                prv = cur
                cur = nxt
                nxt = self.img[i][j+1] == 255

                n = None

                if not cur:
                    # On wall
                    continue

                if prv:

                    if nxt:
                        # PATH PATH PATH
                        if (self.img[i+1][j] == 255) or (self.img[i-1][j] == 255):
                            n = Maze.Node((i,j))
                            leftnode.neighbours[1] = n
                            n.neighbours[3] = leftnode
                            leftnode = n

                    else:
                        # PATH PATH WALL
                        n = Maze.Node((i,j))
                        leftnode.neighbours[1] = n
                        n.neighbours[3] = leftnode
                        leftnode = None

                else:

                    if nxt:
                        # WALL PATH PATH
                        n = Maze.Node((i,j))
                        leftnode = n

                    else:
                        # WALL PATH WALL
                        if (self.img[i+1][j] == 0) or (self.img[i-1][j] == 0):
                            n = Maze.Node((i,j))

                if n != None:
                    if (self.img[i-1][j] == 255):
                        t = topnodes[j]
                        t.neighbours[0] = n
                        n.neighbours[2] = t

                    if (self.img[i+1][j] == 255):
                        topnodes[j] = n

                    else:
                        topnodes[j] = None
        #Bottom node
        for i in range(self.width):
            if self.img[self.height-1][i] == 255:
                n = Maze.Node((self.height-1,i))
                self.end_node = n
                t = topnodes[i]
                t.neighbours[2] = n
                n.neighbours[0] = t

        print("Nodes have been created!")
        self.img = cv2.cvtColor(self.img, cv2.COLOR_GRAY2BGR)

    def showImg(self):
        cv2.imshow("Original", self.img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.stopAll()

    def showResized(self):
        res = cv2.resize(self.img, (2*self.width, 2*self.height), interpolation = cv2.INTER_CUBIC)
        cv2.imshow("Resized", res)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.stopAll()

    def stopAll(self):
        cv2.destroyAllWindows()
        exit()

    def solve(self, alg):

        self.queue = {}
        self.from_dict = {}
        self.visited = []

        self.queue[self.start_node] = 0

        self.path = []
        self.Finished = False

        if alg.lower() == "spacefill":
            Maze.spacefill(self)
        elif alg.lower() == "depthfirst":
            Maze.depthFirst(self)

        for i in range(len(self.path)):
            if i != len(self.path)-1:
                node_1 = self.path[i]
                node_2 = self.path[i+1]
                cv2.line(self.img, (node_1.y, node_1.x), (node_2.y, node_2.x), self.foundPathColorBGR)

        self.showResized()
        if cv2.waitKey(0) & 0xFF == ord('q'):
            self.stopAll()

        

    def spacefill(self):

        print("\nSolving...")

        while not self.Finished:

            selected = min(self.queue.values())

            for i in self.queue.keys():
                if self.queue[i] == selected:
                    selected = i
                    break

            if selected == self.end_node:
                nextnode = self.end_node
                while True:
                    self.path.insert(0, nextnode)
                    if nextnode == self.start_node:
                        break
                    nextnode = self.from_dict[nextnode]

                self.Finished = True
                break

            for node in selected.neighbours:
                if node != None:
                    if node not in self.visited:
                        cv2.line(self.img, (selected.y, selected.x), (node.y, node.x), self.notFoundColorBGR)
                        self.queue[node] = self.queue[selected]+1
                        self.from_dict[node] = selected

            self.visited.append(selected)

            del self.queue[selected]

            #self.showImg()
            self.showResized()

        print("\nSolved.")

    def depthFirst(self):

        print("\nSolving...")

        while not self.Finished:

##            selected = max([i for i in self.queue.values() if float(i).is_integer()])
            selected = min(self.queue.values())

            for i in self.queue.keys():
                if self.queue[i] == selected:
                    selected = i
                    break

            if selected == self.end_node:
                nextnode = self.end_node
                while True:
                    self.path.insert(0,nextnode)
                    if nextnode == self.start_node:
                        break
                    nextnode = self.from_dict[nextnode]
                
                self.Finished = True
                break

            for node in selected.neighbours:
                if node != None:
                    if node not in self.visited:
                        cv2.line(self.img, (selected.y, selected.x), (node.y, node.x), self.notFoundColorBGR)
                        self.queue[node] = self.queue[selected] - 1
                        self.from_dict[node] = selected

            self.visited.append(selected)

            del self.queue[selected]

            self.showResized()


# img = cv2.imread("combo400.png", 0)
# height, width = img.shape[:2]
# res = cv2.resize(img, (2*width, 2*height), interpolation = cv2.INTER_CUBIC)
# cv2.imshow("Resized", res)
# cv2.waitKey(0)

if __name__ == '__main__':
    maze = Maze("small.png")
    maze.createNodes()
    #maze.showResized()
    maze.solve("depthfirst")