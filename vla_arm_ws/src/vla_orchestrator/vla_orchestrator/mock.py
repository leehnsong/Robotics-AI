# mock 데이터 — perception/planner/control 서비스 없을 때 사용

MOCK_OBJECTS = [
    {"name": "red_block",  "position": {"x": 0.45, "y": 0.15,  "z": 0.43}},
    {"name": "blue_plate", "position": {"x": 0.55, "y": -0.20, "z": 0.41}},
]

MOCK_PLAN = {
    "plan": [
        {"action": "open_gripper",  "target": ""},
        {"action": "pick",          "target": "red_block"},
        {"action": "place",         "target": "blue_plate"},
        {"action": "close_gripper", "target": ""},
        {"action": "move_home",     "target": ""},
    ]
}

MOCK_ROBOT_STATE = {"gripper": "open", "holding": None}