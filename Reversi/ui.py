
import game


class Interface(game.ReversiGameLogic):
    def drawBoard(self):
        FIRST="  1 2 3 4 5 6 7 8"
        ORDER=['A','B','C','D','E','F','G','H']
        print(FIRST)
        x=0
        for i in ORDER:
            print(i,self.board[x][0],self.board[x][1],self.board[x][2],self.board[x][3],self.board[x][4],self.board[x][5],self.board[x][6],self.board[x][7])
            x=x+1    
#draw a new board            
            
    def getPlayerMove(self):
        DIGIT = '1 2 3 4 5 6 7 8'.split()
        ORDER_MOVE={'A':1,'B':2,'C':3,'D':4,'E':5,'F':6,'G':7,'H':8}
        while True:
            print("Enter move:")
            move = input().upper()
            if len(move)==2 and (move[0] in ORDER_MOVE) and (move[1] in DIGIT):
                x = int(ORDER_MOVE[move[0]])-1
                y = int(move[1])-1
                if self.isValidMove(x,y) == False:
                    continue
                else:
                    break
            else:
                print("That is not a valid move.")
        return [x,y]
        
    
    def printValid(self):
        ORDER={1:'A',2:'B',3:'C',4:'D',5:'E',6:'F',7:'G',8:'H'}
        printValid=[]
        for x in range(8):
            for y in range(8):
                if self.isValidMove(x, y)!=False:
                    printValid.append(ORDER[x+1]+str(y+1)+', ')
        count=1
        while count < len(printValid):
            printValid[0] = printValid[0] + printValid[count]
            count += 1
        print('Legal Moves: ',printValid[0])
		

