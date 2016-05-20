import random
import sys

def drawBoard(board):
    # This function prints out the board that it was passed. Returns None.
    HLINE = '  +---+---+---+---+---+---+---+---+'
    VLINE = '  |   |   |   |   |   |   |   |   |'

    print('    1   2   3   4   5   6   7   8')
    print(HLINE)
    for y in range(8):
        print(VLINE)
        print(y+1, end=' ')
        for x in range(8):
            print('| %s' % (board[x][y]), end=' ')
        print('|')
        print(VLINE)
        print(HLINE)


def resetBoard(board):
    # Blanks out the board it is passed, except for the original starting position.
    for x in range(8):
        for y in range(8):
            board[x][y] = ' '

    # Starting pieces:
    board[3][3] = 'B'
    board[3][4] = 'W'
    board[4][3] = 'B'
    board[4][4] = 'W'


def getNewBoard():
    # Creates a brand new, blank board data structure.
    board = []
    for i in range(8):
        board.append([' '] * 8)

    return board


def isValidMove(board, tile, xstart, ystart):
    # Returns False if the player's move on space xstart, ystart is invalid.
    # If it is a valid move, returns a list of spaces that would become the player's if they made a move here.
    if board[xstart][ystart] != ' ' or not isOnBoard(xstart, ystart):
        return False

    board[xstart][ystart] = tile # temporarily set the tile on the board.

    if tile == 'B':
        otherTile = 'W'
    else:
        otherTile = 'B'

    tilesToFlip = []
    for xdirection, ydirection in [[0, 1], [1, 1], [1, 0], [1, -1], [0, -1], [-1, -1], [-1, 0], [-1, 1]]:
        x, y = xstart, ystart
        x += xdirection # first step in the direction
        y += ydirection # first step in the direction
        if isOnBoard(x, y) and board[x][y] == otherTile:
            # There is a piece belonging to the other player next to our piece.
            x += xdirection
            y += ydirection
            if not isOnBoard(x, y):
                continue
            while board[x][y] == otherTile:
                x += xdirection
                y += ydirection
                if not isOnBoard(x, y): # break out of while loop, then continue in for loop
                    break
            if not isOnBoard(x, y):
                continue
            if board[x][y] == tile:
                # There are pieces to flip over. Go in the reverse direction until we reach the original space, noting all the tiles along the way.
                while True:
                    x -= xdirection
                    y -= ydirection
                    if x == xstart and y == ystart:
                        break
                    tilesToFlip.append([x, y])

    board[xstart][ystart] = ' ' # restore the empty space
    if len(tilesToFlip) == 0: # If no tiles were flipped, this is not a valid move.
        return False
    return tilesToFlip


def isOnBoard(x, y):
    # Returns True if the coordinates are located on the board.
    return x >= 0 and x <= 7 and y >= 0 and y <=7


def getBoardWithValidMoves(board, tile):
    # Returns a new board with . marking the valid moves the given player can make.
    dupeBoard = getBoardCopy(board)

    for x, y in getValidMoves(dupeBoard, tile):
        dupeBoard[x][y] = '.'
    return dupeBoard


def getValidMoves(board, tile):
    # Returns a list of [x,y] lists of valid moves for the given player on the given board.
    validMoves = []

    for x in range(8):
        for y in range(8):
            if isValidMove(board, tile, x, y) != False:
                validMoves.append([x, y])
    return validMoves


def getScoreOfBoard(board):
    # Determine the score by counting the tiles. Returns a dictionary with keys 'B' and 'W'.
    bscore = 0
    wscore = 0
    for x in range(8):
        for y in range(8):
            if board[x][y] == 'B':
                bscore += 1
            if board[x][y] == 'W':
                wscore += 1
    return {'B':bscore, 'W':wscore}


def enterPlayer1Tile():
    # Lets the player type which tile they want to be.
    # Returns a list with the player's tile as the first item, and the computer's tile as the second.
    tile = ''
    while not (tile == 'B' or tile == 'W'):
        print('Do you want to be B or W?')
        tile = input().upper()

    # the first element in the tuple is the player's tile, the second is the computer's tile.
    if tile == 'B':
        return ['B', 'W']
    else:
        return ['B', 'W']


def whoGoesFirstComputer():
    # Randomly choose the player who goes first.
    if random.randint(0, 1) == 0:
        return 'computer'
    else:
        return 'player'

def whoGoesFirstHuman():
    # Randomly choose the player who goes first.
    if random.randint(0, 1) == 0:
        return 'player1'
    else:
        return 'player2'


def playAgain():
    # This function returns True if the player wants to play again, otherwise it returns False.
    print('Do you want to play again? (yes or no)')
    return input().lower().startswith('y')


def makeMove(board, tile, xstart, ystart):
    # Place the tile on the board at xstart, ystart, and flip any of the opponent's pieces.
    # Returns False if this is an invalid move, True if it is valid.
    tilesToFlip = isValidMove(board, tile, xstart, ystart)

    if tilesToFlip == False:
        return False

    board[xstart][ystart] = tile
    for x, y in tilesToFlip:
        board[x][y] = tile
    return True


def getBoardCopy(board):
    # Make a duplicate of the board list and return the duplicate.
    dupeBoard = getNewBoard()

    for x in range(8):
        for y in range(8):
            dupeBoard[x][y] = board[x][y]

    return dupeBoard


def isOnCorner(x, y):
    # Returns True if the position is in one of the four corners.
    return (x == 0 and y == 0) or (x == 7 and y == 0) or (x == 0 and y == 7) or (x == 7 and y == 7)


def getPlayer1Move(board, player1Tile):
    # Let the player type in their move.
    DIGITS1TO8 = '1 2 3 4 5 6 7 8'.split()
    while True:
        print('Enter your move, or type quit to end the game.')
        move = input().lower()
        if move == 'quit':
            return 'quit'
        if len(move) == 2 and move[0] in DIGITS1TO8 and move[1] in DIGITS1TO8:
            x = int(move[0]) - 1
            y = int(move[1]) - 1
            if isValidMove(board, player1Tile, x, y) == False:
                continue
            else:
                break
        else:
            print('That is not a valid move. Type the x digit (1-8), then the y digit (1-8).')
            print('For example, 81 will be the top-right corner.')

    return [x, y]

def getPlayer2Move(board, player2Tile):
    # Let the player type in their move.
    # Returns the move as [x, y] 
    DIGITS1TO8 = '1 2 3 4 5 6 7 8'.split()
    while True:
        print('Enter your move, or type quit to end the game.')
        move = input().lower()
        if move == 'quit':
            return 'quit'
        if len(move) == 2 and move[0] in DIGITS1TO8 and move[1] in DIGITS1TO8:
            x = int(move[0]) - 1
            y = int(move[1]) - 1
            if isValidMove(board, player2Tile, x, y) == False:
                continue
            else:
                break
        else:
            print('That is not a valid move. Type the x digit (1-8), then the y digit (1-8).')
            print('For example, 81 will be the top-right corner.')

    return [x, y]


def getComputerMove(board, computerTile):
    # Given a board and the computer's tile, determine where to
    # move and return that move as a [x, y] list.
    possibleMoves = getValidMoves(board, computerTile)

    # randomize the order of the possible moves
    random.shuffle(possibleMoves)

    # always go for a corner if available.
    for x, y in possibleMoves:
        if isOnCorner(x, y):
            return [x, y]

    # Go through all the possible moves and remember the best scoring move
    bestScore = -1
    for x, y in possibleMoves:
        dupeBoard = getBoardCopy(board)
        makeMove(dupeBoard, computerTile, x, y)
        score = getScoreOfBoard(dupeBoard)[computerTile]
        if score > bestScore:
            bestMove = [x, y]
            bestScore = score
    return bestMove


def showPoints(player1Tile, computerTile):
    # Prints out the current score.
    scores = getScoreOfBoard(mainBoard)
    print('You have %s points. The computer has %s points.' % (scores[player1Tile], scores[computerTile]))
	
def showPointsHuman(player1Tile, player2Tile):
    # Prints out the current score.
    scores = getScoreOfBoard(mainBoard)
    print('Player 1 has %s points. Player2 has %s points.' % (scores[player1Tile], scores[player2Tile]))



print('Welcome to Reversi!')

while True:
    # Reset the board and game.
	Answer=input('Who do want to play, Computer or Human')
	if Answer=='Computer':
    # Reset the board and game.
		mainBoard = getNewBoard()
		resetBoard(mainBoard)
		player1Tile, computerTile = enterPlayer1Tile()
		turn = whoGoesFirstComputer()
		print('The ' + turn + ' will go first.')

		while True:
			if turn == 'player':
				# Player's turn.
				drawBoard(mainBoard)
				showPoints(player1Tile, computerTile)
				move = getPlayer1Move(mainBoard, player1Tile)
				if move == 'quit':
					print('Thanks for playing!')
					sys.exit() # terminate the program
				else:
					makeMove(mainBoard, player1Tile, move[0], move[1])

				if getValidMoves(mainBoard, computerTile) == []:
					break
				else:
					turn = 'computer'

			else:
				# Computer's turn.
				drawBoard(mainBoard)
				showPoints(player1Tile, computerTile)
				input('Press Enter to see the computer\'s move.')
				x, y = getComputerMove(mainBoard, computerTile)
				makeMove(mainBoard, computerTile, x, y)

				if getValidMoves(mainBoard, player1Tile) == []:
					break
				else:
					turn = 'player'

		# Display the final score.
		drawBoard(mainBoard)
		scores = getScoreOfBoard(mainBoard)
		print('B scored %s points. W scored %s points.' % (scores['B'], scores['W']))
		if scores[player1Tile] > scores[computerTile]:
			print('You beat the computer by %s points! Congratulations!' % (scores[player1Tile] - scores[computerTile]))
		elif scores[player1Tile] < scores[computerTile]:
			print('You lost. The computer beat you by %s points.' % (scores[computerTile] - scores[player1Tile]))
		else:
			print('The game was a tie!')

		if not playAgain():
			break
	elif Answer=='Human':
	
	
	
	
		mainBoard = getNewBoard()
		resetBoard(mainBoard)
		player1Tile, player2Tile = enterPlayer1Tile()
		turn = whoGoesFirstHuman()
		print('The ' + turn + ' will go first.')
	
		while True:
			if turn == 'player1':
				# Player's turn.
				
				drawBoard(mainBoard)
				showPointsHuman(player1Tile, player2Tile)
				move = getPlayer1Move(mainBoard, player1Tile)
				if move == 'quit':
					print('Thanks for playing!')
					sys.exit() # terminate the program
				else:
					makeMove(mainBoard, player1Tile, move[0], move[1])

				if getValidMoves(mainBoard, player2Tile) == []:
					break
				else:
					turn = 'player2'

			else:
				# player2 turn.
				drawBoard(mainBoard)
				showPointsHuman(player1Tile, player2Tile)
				x, y = getPlayer2Move(mainBoard, player2Tile)
				makeMove(mainBoard, player2Tile, x, y)

				if getValidMoves(mainBoard, player1Tile) == []:
					break
				else:
					turn = 'player1'

		# Display the final score.
		drawBoard(mainBoard)
		scores = getScoreOfBoard(mainBoard)
		print('B scored %s points. W scored %s points.' % (scores['B'], scores['W']))
		if scores[player1Tile] > scores[player2Tile]:
			print('Player 1 beat Player 2 by %s points! Congratulations!' % (scores[player1Tile] - scores[computerTile]))
		elif scores[player1Tile] < scores[player2Tile]:
			print('You lost. The computer beat you by %s points.' % (scores[player2Tile] - scores[player1Tile]))
		else:
			print('The game was a tie!')

		if not playAgain():
			break
	else:
		print("Not a Valid Input")