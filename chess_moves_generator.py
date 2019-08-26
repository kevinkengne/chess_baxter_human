#!/usr/bin/env python3.6
"""" Generates chess_moves for Baxter according to python-chess module and Stockfish engine.
"""

import rospy
import sys
from std_msgs.msg import String
import json
import chess
import chess.engine
import time

engine = chess.engine.SimpleEngine.popen_uci('/usr/local/bin/stockfish')
board = chess.Board()
    


# Integer representation of each sqaure according to the python-chess module
squares_integers = {
    'a1': 0,
    'b1': 1,
    'c1': 2,
    'd1': 3,
    'e1': 4,
    'f1': 5,
    'g1': 6,
    'h1': 7,
    'a2': 8,
    'b2': 9,
    'c2': 10,
    'd2': 11,
    'e2': 12,
    'f2': 13,
    'g2': 14,
    'h2': 15,
    'a3': 16,
    'b3': 17,
    'c3': 18,
    'd3': 19,
    'e3': 20,
    'f3': 21,
    'g3': 22,
    'h3': 23,
    'a4': 24,
    'b4': 25,
    'c4': 26,
    'd4': 27,
    'e4': 28,
    'f4': 29,
    'g4': 30,
    'h4': 31,
    'a5': 32,
    'b5': 33,
    'c5': 34,
    'd5': 35,
    'e5': 36,
    'f5': 37,
    'g5': 38,
    'h5': 39,
    'a6': 40,
    'b6': 41,
    'c6': 42,
    'd6': 43,
    'e6': 44,
    'f6': 45,
    'g6': 46,
    'h6': 47,
    'a7': 48,
    'b7': 49,
    'c7': 50,
    'd7': 51,
    'e7': 52,
    'f7': 53,
    'g7': 54,
    'h7': 55,
    'a8': 56,
    'b8': 57,
    'c8': 58,
    'd8': 59,
    'e8': 60,
    'f8': 61,
    'g8': 62,
    'h8': 63
}



def chess_moves_publishing(boards):
    
    pub = rospy.Publisher('chess_next_moves', String, queue_size=10)
    rospy.init_node('chess_moves', anonymous=True)
    
    while not board.is_game_over():
        while not rospy.is_shutdown():
            
            result = engine.play(boards, chess.engine.Limit(time=0.100))
            next_move = str(result.move)
            recipient_square = boards.piece_at(squares_integers[next_move[2:]])
            boards.push(result.move)
            pieceCount = update_piece_count(boards)
            print(pieceCount)
            #print(next_move)
            #print(next_move[2:])
            #print(board.piece_at(squares_integers[next_move[2:]]))
            if (recipient_square is None):
                rospy.loginfo(next_move)
                pub.publish(next_move)
            else:
                rospy.loginfo(next_move + 's')
                #pieceCount -= 1
                pub.publish(next_move + 's')
            print("waiting on human move...")
            rospy.sleep(235)
            print(pieceCount)
            generate_human_move(pieceCount)
            rospy.sleep(5)

    engine.quit()

def update_piece_count(boards):
    pieceCount = 0
    # dynamically updating the number of pieces on the board
    for i in range(64):
        if boards.piece_at(i) is not None:
            pieceCount += 1
    return pieceCount

def generate_human_move(pieceCount):
    # Generates human move updates the board
    # opening the json file containing the empty and full squares
    squares = {}
    human_move = ""
    with open('squares.json') as json_file:
        squares = json.load(json_file)
    if len(squares["full"]) == pieceCount: 
        donor_square = determine_donor_square(squares["empty"])
        recipient_square = determine_recipient_square(squares["full"])      
        human_move = determine_move(donor_square, recipient_square)
        print(human_move)
    elif len(squares["full"]) == pieceCount - 1:
        #pieceCount -= 1
        donor_square = determine_donor_square(squares["empty"])
        human_move = determine_capturing_move(donor_square)
        print(human_move)
    else:
        print("cheated")
        sys.exit(0)
        
    move = chess.Move.from_uci(human_move)
    if move in board.legal_moves:
        print("pushed")
        board.push(move)
    return pieceCount
    


def determine_move(donor_square, recipient_square):
    # determine move string (ex: e2e4) based on donor and recipient square
    squares_labels = {v: k for k, v in squares_integers.items()}
    donor_string = squares_labels[donor_square]
    recipient_string = squares_labels[recipient_square]
    move = donor_string + recipient_string
    return move

def determine_capturing_move(donor_square):
    # determine move string based on donor and captured piece
    squares_labels = {v: k for k, v in squares_integers.items()}
    donor_string = squares_labels[donor_square]
    legal_list = []
    for move in board.legal_moves:
        legal_list.append(move.uci())
    for move in legal_list:
        if move[:2] == donor_string and board.piece_at(squares_integers[move[2:]]) is not None:
            return move 

def determine_donor_square(empty_list_previous):
    # determine donor square based on image analysis node
    for i in range(len(empty_list_previous)):
        if board.piece_at(empty_list_previous[i]) is not None:            
            return empty_list_previous[i]

   
            

def determine_recipient_square(full_list_previous):
    # determine recipient square based on image analysis node
    for i in range(len(full_list_previous)):
        if board.piece_at(full_list_previous[i]) is None:
            return full_list_previous[i] 



if __name__ == '__main__':
   chess_moves_publishing(board)
   #while True: 
   #    time.sleep(180)
   #    pieceCount = generate_human_move(pieceCount)
     #  print(pieceCount)
       
       
   