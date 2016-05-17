import ui
import game




if __name__ == "__main__":
    while True:
        mainBoard = ui.Interface()
        mainBoard.resetBoard()
        while True:
            mainBoard.drawBoard()
            scores = mainBoard.getScoreOfBoard()
            print("Score: Black %s, White %s "%(scores['b'], scores['w']))
            print("It is black's turn to play")
            mainBoard.getturn('b')
            mainBoard.printValid()
            move=mainBoard.getPlayerMove()
            mainBoard.makeMove(xstart=move[0],ystart=move[1])
            if mainBoard.getValidMoves()==[]:
                break
            mainBoard.drawBoard()
            scores = mainBoard.getScoreOfBoard()
            print("Score: Black %s, White %s "%(scores['b'], scores['w']))        
            print("It is white's turn to play")
            mainBoard.getturn('w')
            mainBoard.printValid()
            move=mainBoard.getPlayerMove()
            mainBoard.makeMove(xstart=move[0],ystart=move[1])
            if mainBoard.getValidMoves()==[]:
                break
        mainBoard.drawBoard()
        scores = mainBoard.getScoreOfBoard()
        print ('b scored %s points. w scored %s points.'%(scores['b'], scores['w']))
        points = scores['b'] - scores['w']
        if points>0:
            print("Black beats the white by %s points!"%(points))
        elif points<0:
            print("White beats the black by %s points!"%(0-points))
        else:
            print("The game was a tie.")
        play = False
        while True:
            print("Do you want to play again ?  (y/n)")
            if input().lower()=='y':
                play = True
                break
            elif input().lower()=='n':
                play == False
                break
        if play==False:
            break  
        
