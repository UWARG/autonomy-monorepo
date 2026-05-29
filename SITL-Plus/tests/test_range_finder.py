import range_finder as range_finder


def test_range_finder_init(bullet_connect):
    range_finder_obj=range_finder.Range_Finder(port=6004,direction=[0,0,-1],dist=100)
    assert range_finder_obj.port==6004
    assert range_finder_obj.direction==[0,0,-1]
    assert range_finder_obj.dist==100

def test_range_finder_update(bullet_connect,range_finder_obj):
    range_finder_obj.update()
    assert range_finder_obj.range is not None

