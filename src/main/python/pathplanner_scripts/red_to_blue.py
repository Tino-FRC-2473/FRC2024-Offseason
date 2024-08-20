import json

REL_PATH = "src\\main\\deploy\\pathplanner\\paths"

def dump_json_object(path: str, json_object) -> None:
    """
    Dumps the json object to the specified path.
    """
    with open(path, 'w') as outfile:
        json.dump(outfile, json_object)

def load_json_object(path: str) -> dict:
    """
    Loads the json object from the specified path.

    Returns the json object.
    """
    with open(path, 'r') as openfile:
        return json.load(openfile)


def swap_color_paths(paths: list, blue_prefix:str="B", red_prefix:str="R") -> bool:
    """
    B1 --> R1, etc.
    leaves the relative timing of all the commands, rotation targets, etc. THE SAME! adjust as necessary

    starting poses:

    B1: 0.69, 6.74, __   | R1: 15.89, 6.76, __
    B2: 1.34, 5.55, __   | R2: 15.19, 5.58, __
    B3: 0.69, 4.35, __   | R3: 15.89, 4.39, __

    * assumes starting and ending positions are the same.

    transfer headings + rotations: 180 - heading/rotation
    keep rotation targets and event markers at the same spot

    """

    for path in paths:
        if path.index(blue_prefix) == 0 or path.index(red_prefix) == 0:
            blue_path = REL_PATH + path
            red_path = REL_PATH + red_prefix + str(paths.index(red_prefix) + 1)

            swap_objects(red_path, blue_path)
            return True
        else:
            raise ValueError("File path does not start with specified blue or red prefix.")
            return False


    return True

def swap_objects(blue_path: str, red_path: str) -> None:
    #swap to red
    blue_object: dict = load_json_object(blue_path)
    red_object: dict = load_json_object(red_path)

    #
    apply_swap_to_json(blue_object, red_path, True)
    apply_swap_to_json(red_object, blue_path, False)




def apply_swap_to_json(json_object, mirror_path: str, is_blue: bool) -> None:
    waypoints: list = list(json_object["waypoints"])
    json_object["waypoints"] = convert_waypoint_data(waypoints, is_blue)
    convert_rotation_heading(json_object)
    dump_json_object(mirror_path, json_object)

def convert_waypoint_data(waypoints: list, is_blue: bool) -> list:
    """
    Converts the waypoint data to the opposite side of the field.

    Returns the new waypoints list.
    """
    temp = waypoints
    for waypoint in temp:
        x_pos = waypoint["anchor"]["x"]
        y_pos = waypoint["anchor"]["y"]

        (x_pos, y_pos) = convert_xypos(x_pos, y_pos, is_blue)

        waypoint["anchor"]["x"] = x_pos
        waypoint["anchor"]["y"] = y_pos
    return temp

def convert_rotation_heading(json_obj: dict) -> None:
    """
    Converts the rotation and heading of the json object to the opposite side of the field.

    """
    json_obj["goalEndState"]["heading"] = 180 - json_obj["goalEndState"]["heading"]
    json_obj["previewStartingState"]["rotation"] = 180 - json_obj["previewStartingState"]["rotation"]

def convert_xypos(x: float, y: float, is_blue: bool) -> tuple[float, float]:
    """
    Converts the X and Y positions based on if its from the blue side or the red side.

    Returns a tuple of the x and y value.
    """
    # TODO: Implement value converting
    if is_blue:
        pass
    else:
        pass

    return (0, 0)
