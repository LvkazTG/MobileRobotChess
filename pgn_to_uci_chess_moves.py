import chess.pgn
import os

board = chess.Board()
pgn_file = open("LvkazTG_vs_NRPL613_2021.05.25.pgn", "r")

game = chess.pgn.read_game(pgn_file)

print(game.headers["Event"])

board = game.board()

for move in game.mainline_moves():
    print(move.uci())
    board.push(move)

print(board)

