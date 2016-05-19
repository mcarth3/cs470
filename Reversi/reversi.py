import ui
import game
import random
import sys


if __name__ == "__main__":
	Answer=input('Who do you want to play, Computer or Human? ')
	if Answer =='Human':
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
	elif Answer=='Computer':
		while True:
    # refresh the board and game.
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
				print("It is black's turn to play")
				input('Press Enter to see the computer\'s move.')
				mainBoard.getturn('w')
				
				possibleMoves = mainBoard.getValidMoves()
				
	# randomize the order of the possible moves
				random.shuffle(possibleMoves)
				DIGIT = '1 2 3 4 5 6 7 8'.split()
				ORDER_MOVE={'A':1,'B':2,'C':3,'D':4,'E':5,'F':6,'G':7,'H':8}
				len(possibleMoves)==2 and (possibleMoves[0] in ORDER_MOVE) and (possibleMoves[1] in DIGIT)
				move=possibleMoves[0]
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
						
	else:
		print("Invalid Entry")
		
		
	