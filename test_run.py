# Ignore this file.... it was just an early thought on how to structure the test


def test_run(test_list):
    '''
        Parameters:
        ----------
            test_list (list) : list of tuples containing the wanted tests.
        
            The list is given in the following format:
                [ (laps_x1, speed_x1), (laps_x2, speed_x2), ... , (laps_xn, speed_xn) ]
    '''

    test_result = []

    for test in test_list:
        laps = test[0]
        speed = test[1]
        result = perform_test(laps, speed)
        test_result.append(result)
    
    go_to_rest()

    return test_result

def perform_test(laps, speed):

    lap_counter = 0
    robot = init_robot()
    sensor_values = []

    while (lap_counter < laps):
        img = robot.get_image()
        linear, angular = robot.calculate_velocity(img)
        robot.move(linear, angular)
        sensor_values.append(robot.read_sensors())
    
    return sensor_values

def go_to_rest():

    robot = init_robot()
    in_resting_place = False

    while (!in_resting_place):
        img = robot.get_image()
        linear, angular
        in_resting_place = find_resting_place(img)

        
'''
    To make it easier to know some segments of the road, lets mark them with different colors. 
    Then it would be easy to find different portions of the track. 

    Some kind of marking for the start of difficult section, and some kind of marking at the end 
    of the difficult section. 

    Some kinf of marking at the start of the track. 

    Some kind of marking by the resting place of the robot. 
'''
