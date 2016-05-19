import ui



class ReversiGameLogic:
    
    def __init__(self):
        self.board = []
        for i in range(8):
            self.board.append([" "]*8)
#set a empty board           
            
    def getturn(self,turn):
        self.turn = turn
#set player's turn        
        
    def isOnBoard(self,x,y):
        return (x>=0 and x<=7 and y>=0 and y<=7)
#determine whether the move is on board or not
        
    def isOnCorner(self,x,y):
        return (x==0 or x==7) and (y==0 or y==7)
#determine whether the move is on corner or not
    
    def resetBoard(self):
        for x in range(8):
            for y in range(8):
                self.board[x][y] = "."
        self.board[3][3] = "w"
        self.board[3][4] = "b"
        self.board[4][3] = "b"
        self.board[4][4] = "w" 
#reset the board

	
    def makeMove(self,xstart,ystart):
        #Place the tile on the board at xstart and ystart, and flip any of the opponent's pieces
        tilesToFlip = self.isValidMove(xstart,ystart)    
        if tilesToFlip == False:
            return False
        self.board[xstart][ystart] = self.tile
        for x, y in tilesToFlip:
            self.board[x][y] = self.tile
        return True
#make a move on board

    def getScoreOfBoard(self):
        bscore = 0
        wscore = 0
        for x in range(8):
            for y in range(8):
                if self.board[x][y] == 'b':
                    bscore += 1
                if self.board[x][y] == 'w':
                    wscore += 1
        return {'b':bscore, 'w':wscore}
#get the score from board    
    
                
    def getValidMoves(self):
        validMoves = []
        for x in range(8):
            for y in range(8):
                if self.isValidMove(x, y) != False:
                    validMoves.append([x, y])
        return validMoves
#gather the valid moves to a list in order to move    
    
    def isValidMove(self, xstart, ystart):
        if (self.board[xstart][ystart] != '.') or (not self.isOnBoard(xstart,ystart)):
            return False
        if self.turn=='b':
            self.tile='b'
            self.otherTile='w'
        elif self.turn=='w':
            self.tile='w'
            self.otherTile='b'
        self.board[xstart][ystart] = self.tile
        tilesToFlip = []
        direction = [[0,1],[1,1],[1,0],[1,-1],[0,-1],[-1,-1],[-1,0],[-1,1]]
        for xd, yd in direction:
            x, y = xstart, ystart
            x += xd
            y += yd
            if self.isOnBoard(x, y) and self.board[x][y]==self.otherTile:
                x += xd
                y += yd
                if not self.isOnBoard(x, y):
                    continue
                while self.board[x][y] == self.otherTile:
                    x += xd
                    y += yd
                    if not self.isOnBoard(x, y):
                        break
                if not self.isOnBoard(x, y):
                    continue
                if self.board[x][y] == self.tile:
                    while True:
                        x -= xd
                        y -= yd
                        if x==xstart and y==ystart:
                            break
                        tilesToFlip.append([x, y])
        self.board[xstart][ystart] = '.' # back to before
        if len(tilesToFlip)==0:
            return False
        return tilesToFlip
#to see whether this move is a valid move or not


