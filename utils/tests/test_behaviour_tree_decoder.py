import pytest

from src.behaviour_tree_decoder import BehaviourTreeDecoder

TABLE = ["T,0,2,Takeoff", "T,1,2,Lapping", "T,2,3,Mission", "T,3,-1,KillSwitch"]


def make_decoder():
    decoder = BehaviourTreeDecoder()
    for line in TABLE:
        decoder.handle_message(line)
    return decoder


def test_current_path_starts_at_root():
    decoder = make_decoder()
    decoder.handle_message("S,1")
    assert decoder.current_path() == ["KillSwitch", "Mission", "Lapping"]


def test_render_marks_running_branch():
    decoder = make_decoder()
    decoder.handle_message("S,1")
    assert decoder.render() == "\n".join(
        ["* KillSwitch", "    * Mission", "          Takeoff", "        * Lapping"]
    )


def test_load_table_from_file(tmp_path):
    path = tmp_path / "behaviour_tree.txt"
    path.write_text("\n".join(TABLE) + "\n")
    decoder = BehaviourTreeDecoder()
    decoder.load_table(path)
    decoder.handle_message("S,0")
    assert decoder.current_path() == ["KillSwitch", "Mission", "Takeoff"]


def test_unknown_running_node_raises():
    decoder = make_decoder()
    decoder.handle_message("S,9")
    with pytest.raises(ValueError):
        decoder.current_path()