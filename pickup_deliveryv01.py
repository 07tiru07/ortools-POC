"""Simple Vehicles Routing Problem."""
"""Get routes for all the combinations of the start points."""

from __future__ import print_function
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

from time import sleep 
import math
from itertools import combinations

CAB_CAPACITY = 3


def create_data_model():
    """Stores the data for the problem."""
    data = {}
    data['distance_matrix'] = [[0, 3614, 7213, 7922, 6945, 17943, 25253, 44904, 19478, 12146, 10706], [1978, 0, 8827, 9535, 8558, 14375, 21685, 41336, 17135, 9616, 9900], [8750, 8475, 0, 5111, 2256, 18204, 25514, 45165, 23544, 12430, 5373], [9873, 9599, 4337, 0, 4068, 29259, 36113, 55764, 27327, 8988, 9500], [9158, 8883, 1188, 5519, 0, 19182, 26492, 46143, 31177, 12837, 6351], [15747, 13832, 17938, 20517, 17670, 0, 11351, 31002, 5707, 20716, 14174], [25443, 23528, 27634, 37592, 27366, 11833, 0, 19659, 10617, 30911, 23870], [43364, 41449, 45555, 55513, 45287, 29754, 22700, 0, 28548, 48832, 41791], [18383, 16377, 30329, 27249, 30061, 5714, 8952, 28612, 0, 20567, 19746], [7560, 7230, 14408, 11327, 14139, 20271, 27125, 46776, 18339, 0, 17084], [12745, 10830, 5333, 9858, 7002, 14716, 22026, 41677, 20056, 17176, 0]]
    # data['num_vehicles'] = 2
    # data['starts'] = [1, 3]
    # data['ends'] = [0, 0]
    return data


def print_solution(data, manager, routing, solution):
    """Prints solution on console."""
    max_route_distance = 0
    for vehicle_id in range(data['num_vehicles']):
        index = routing.Start(vehicle_id)
        plan_output = 'Route for vehicle {}:\n'.format(vehicle_id)
        route_distance = 0
        while not routing.IsEnd(index):
            plan_output += ' {} -> '.format(manager.IndexToNode(index))
            previous_index = index
            index = solution.Value(routing.NextVar(index))
            route_distance += routing.GetArcCostForVehicle(
                previous_index, index, vehicle_id)
        plan_output += '{}\n'.format(manager.IndexToNode(index))
        plan_output += 'Distance of the route: {}m\n'.format(route_distance)
        print(plan_output)
        max_route_distance = max(route_distance, max_route_distance)
    print('Maximum of the route distances: {}m'.format(max_route_distance))


def main():
    """Entry point of the program."""
    # Instantiate the data problem.
    data = create_data_model()
    print(data)

    # Running for loop to check for each possible way to find the route 
    print("No of dropoff/pickup points", len(data['distance_matrix']))
    no_of_employees = len(data['distance_matrix']) - 1
    no_of_cabs = math.ceil(no_of_employees / CAB_CAPACITY)
    print("no of employees", no_of_employees)
    print("no of cabs", no_of_cabs)
    ends = [0]*no_of_cabs
    starts = list(range(1,no_of_employees))
    print("starts",starts)
    print("ends",ends)

    list_of_start_combinations = list(combinations(starts,no_of_cabs))
    print("combinations",list_of_start_combinations)

    data['ends'] = ends
    data['num_vehicles'] = no_of_cabs

    for start in list_of_start_combinations:
        data['starts'] = start
        print("start", start)
        # Create the routing index manager.
        manager = pywrapcp.RoutingIndexManager(len(data['distance_matrix']),
                                            data['num_vehicles'], data['starts'],
                                            data['ends'])

        # Create Routing Model.
        routing = pywrapcp.RoutingModel(manager)
        print(routing)


        # Create and register a transit callback.
        def distance_callback(from_index, to_index):
            """Returns the distance between the two nodes."""
            # Convert from routing variable Index to distance matrix NodeIndex.
            from_node = manager.IndexToNode(from_index)
            to_node = manager.IndexToNode(to_index)
            return data['distance_matrix'][from_node][to_node]

        transit_callback_index = routing.RegisterTransitCallback(distance_callback)

        # Define cost of each arc.
        routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

        # Add Distance constraint.
        dimension_name = 'Distance'
        routing.AddDimension(
            transit_callback_index,
            0,  # no slack
            50000,  # vehicle maximum travel distance
            True,  # start cumul to zero
            dimension_name)
        distance_dimension = routing.GetDimensionOrDie(dimension_name)
        distance_dimension.SetGlobalSpanCostCoefficient(100)

        # Setting first solution heuristic.
        search_parameters = pywrapcp.DefaultRoutingSearchParameters()
        search_parameters.first_solution_strategy = (
            routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

        # Solve the problem.
        solution = routing.SolveWithParameters(search_parameters)

        # Print solution on console.
        if solution:
            print_solution(data, manager, routing, solution)


if __name__ == '__main__':
    main()