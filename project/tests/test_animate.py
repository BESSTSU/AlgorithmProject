def test_side_pockets(challenge_map):
    pockets = [
        (r, c)
        for r in range(challenge_map.height)
        for c in range(challenge_map.width)
        if challenge_map.is_side_pocket((r, c))
    ]
    # (3,0) is the end of the corridor, not a pocket; doors are colored separately.
    assert pockets == [(4, 6)]
