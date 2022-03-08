from ortools.constraint_solver import pywrapcp
from ortools.constraint_solver import routing_enums_pb2

# Adapted from https://developers.google.com/optimization/routing/vrp

class CityBlock():
    """City block definition"""
    @property
    def width(self):
        """Gets Block size West to East"""
        return 228/2

    @property
    def height(self):
        """Gets Block size North to South"""
        return 80


class DataProblem():
    """Store the data for the problem"""
    def __init__(self):
        """Initializes the data for the problem"""
        self._num_vehicles = 4

        # Locations in block unit
        locations = \
            [ (4, 4), # depot
              (2, 0), (8, 0),  # row 0
              (0, 1), (0, 1),
              (5, 2), (7, 2),
              (3, 3), (6, 3),
              (5, 5), (8, 5),
              (1, 6), (2, 6),
              (3, 7), (6, 7),
              (0, 8), (7, 8) ]
        # Locations in meters using the city block dimension
        city_block = CityBlock()
        self._locations = [ (
            loc[0]*city_block.width,
            loc[1]*city_block.height) for loc in locations ]
        self._depot = 0


    @property
    def num_vehicles(self):
        """Gets number of vehicles"""
        return self._num_vehicles

    @property
    def locations(self):
        """Gets locations"""
        return self._locations

    @property
    def num_locations(self):
        """Gets number of locations"""
        return len(self.locations)

    @property
    def depot(self):
        """Gets depot location index"""
        return self._depot


def manhattan_distance(position_1, position_2):
    """Computes the Manhattan distance between two points"""
    return (abs(position_1[0] - position_2[0]) +
            abs(position_1[1] - position_2[1]))


class CreateDistanceEvaluator(object): # pylint: disable=too-few-public-methods
    """Creates callback to return distance between points."""
    def __init__(self, data, index_manager):
        """Initializes the distance matrix."""
        self._distances = {}
        self._index_manager = index_manager

        # precompute distance between location to have distance callback in O(1)
        for from_node in range(data.num_locations):
            self._distances[from_node] = {}
            for to_node in range(data.num_locations):
                if from_node == to_node:
                    self._distances[from_node][to_node] = 0
                else:
                    self._distances[from_node][to_node] = (
                        manhattan_distance(
                            data.locations[from_node],
                            data.locations[to_node]
                        )
                    )

    def IndexToNode(self, node_index):
        return self._index_manager.IndexToNode(node_index)

    def distance_evaluator(self, from_node_index, to_node_index):
        """Returns the manhattan distance between the two nodes"""
        from_node = self.IndexToNode(from_node_index)
        to_node = self.IndexToNode(to_node_index)
        return self._distances[from_node][to_node]


# add additional constraint
def add_distance_dimension(routing, distance_evaluator_index, maximum_distance=3000):
    """Add Global Span constraint"""
    distance = "Distance"
    routing.AddDimension(
        distance_evaluator_index,
        0,                      # null slack
        maximum_distance,       # maximum distance per vehicle
        True,                   # start cumul to zero
        distance
    )
    distance_dimension = routing.GetDimensionOrDie(distance)
    # Try to minimize the max distance among vehicles.
    # /!\ It doesn't mean the standard deviation is minimized
    distance_dimension.SetGlobalSpanCostCoefficient(100)


class ConsolePrinter():
    """Print solution to console"""
    def __init__(self, data, routing, assignment, index_manager):
        """Initialize the printer"""
        self._data = data
        self._routing = routing
        self._assignment = assignment
        self._index_manager = index_manager


    @property
    def routing(self):
        """Get routing"""
        return self._routing

    @property
    def data(self):
        """Gets problem data"""
        return self._data


    @property
    def assignment(self):
        return self._assignment


    def IndexToNode(self, index):
        return self._index_manager.IndexToNode(index)

    def print(self, silent=False):
        """Prints assignment on console"""
        # Inspect solution.
        total_dist = 0
        for vehicle_id in range(self.data.num_vehicles):

            index = self.routing.Start(vehicle_id)
            plan_output = f'Route for vehicle {vehicle_id}:\n'
            route_dist = 0

            while not self.routing.IsEnd(index):
                node_index = self.IndexToNode(index)
                next_node_index = self.IndexToNode(
                    self.assignment.Value(self.routing.NextVar(index))
                )
                route_dist += manhattan_distance(
                    self.data.locations[node_index],
                    self.data.locations[next_node_index]
                )
                plan_output += f' {node_index} -> '
                index = self.assignment.Value(self.routing.NextVar(index))

            node_index = self.IndexToNode(index)
            total_dist += route_dist
            plan_output += f' {node_index}\n'
            plan_output += f'Distance of the route {vehicle_id}: {route_dist}\n'
            if not silent:
                print(plan_output)

        if not silent:
            print(f'Total Distance of all routes: {total_dist}')

        return total_dist



def solve(with_max_distance_per_vehicle_constraint=False, maximum_distance=3000, silent=False):
    data = DataProblem()

    # Create a RoutingIndexManager
    routing_index_manager = pywrapcp.RoutingIndexManager(data.num_locations, data.num_vehicles, data.depot)
    # Create Routing Model
    routing = pywrapcp.RoutingModel(routing_index_manager)

    # Define weight of each edge
    distance_evaluator = CreateDistanceEvaluator(data, routing_index_manager).distance_evaluator
    distance_evaluator_index = routing.RegisterTransitCallback(distance_evaluator)
    routing.SetArcCostEvaluatorOfAllVehicles(distance_evaluator_index)

    if with_max_distance_per_vehicle_constraint:
        add_distance_dimension(routing, distance_evaluator_index, maximum_distance)

    #Setting first solution heuristic (cheapest addition).
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    )

    # solve the the problem.
    assignment = routing.SolveWithParameters(search_parameters)
    printer = ConsolePrinter(data, routing, assignment, routing_index_manager)
    return printer.print(silent)


def main():
    print("\n===== solution WITHOUT max-distance-per-vehicle-constraint =====\n")
    solve()
    print("\n===== solution WITH max-distance-per-vehicle-constraint =====\n")
    solve(True)




if __name__ == "__main__":
    main()
