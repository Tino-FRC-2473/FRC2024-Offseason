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
    
    rel_path = "src\main\deploy\pathplanner\paths"
    for path in paths:
        if paths.index(blue_prefix) == 0:
            #swap to red
            pass
        elif paths.index(red_prefix) == 0:
            #swap to blue
            pass
        else:
            raise ValueError("File path does not start with specified blue or red prefix.")

    