from src.behaviour_tree_decoder import BehaviourTreeDecoder

decoder = BehaviourTreeDecoder()
decoder.handle_message("T,27,28,Mission")
decoder.handle_message("T,28,-1,KillSwitch")
decoder.handle_message("T,17,27,Lapping")
decoder.handle_message("S,17")

print(decoder.current_path())  # should print ['Lapping', 'Mission', 'KillSwitch']