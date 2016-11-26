import math
from six.moves import xrange
from ortools.constraint_solver import pywrapcp
from ortools.constraint_solver import routing_enums_pb2
import falcon
import json
from wsgiref import simple_server
import numpy as np

def distance(x1, y1, x2, y2):
        # Manhattan distance
        dist = abs(x1 - x2) + abs(y1 - y2)

        return dist

class CreateDistanceCallback(object):
    """Create callback to calculate distances and travel times between points."""

    def __init__(self, locations):
        """Initialize distance array."""
        num_locations = len(locations)
        self.matrix = {}

        for from_node in xrange(num_locations):
            self.matrix[from_node] = {}
            for to_node in xrange(num_locations):
                if from_node == to_node:
                    self.matrix[from_node][to_node] = 0
                else:
                    x1 = locations[from_node][0]
                    y1 = locations[from_node][1]
                    x2 = locations[to_node][0]
                    y2 = locations[to_node][1]
                    self.matrix[from_node][to_node] = distance(x1, y1, x2, y2)

    def Distance(self, from_node, to_node):
        return self.matrix[from_node][to_node]


# In[3]:

# Demand callback
class CreateDemandCallback(object):
    """Create callback to get demands at location node."""

    def __init__(self, demands):
        self.matrix = demands

    def Demand(self, from_node, to_node):
        return self.matrix[from_node]


# Service time (proportional to demand) callback.
class CreateServiceTimeCallback(object):
    """Create callback to get time windows at each location."""

    def __init__(self, demands, time_per_demand_unit):
        self.matrix = demands
        self.time_per_demand_unit = time_per_demand_unit

    def ServiceTime(self, from_node, to_node):
        return self.matrix[from_node] * self.time_per_demand_unit


# Create total_time callback (equals service time plus travel time).
class CreateTotalTimeCallback(object):
    def __init__(self, service_time_callback, dist_callback, speed):
        self.service_time_callback = service_time_callback
        self.dist_callback = dist_callback
        self.speed = speed

    def TotalTime(self, from_node, to_node):
        service_time = self.service_time_callback(from_node, to_node)
        travel_time = self.dist_callback(from_node, to_node) / self.speed
        return service_time + travel_time


# In[4]:

def main(vehicle_capacity, depot_location, customer_locations, customer_demands):
    
    vehicle_capacity = int(vehicle_capacity)
    
    # Create the data.
    data = create_data_array()
    locations = customer_locations
    demands = customer_demands
    num_locations = len(locations)
    depot = 0
    locations.append(depot_location)
    num_vehicles = 5
    search_time_limit = 5000
    output = []
    
    # Create routing model.
    if num_locations > 0:

        # The number of nodes of the VRP is num_locations.
        # Nodes are indexed from 0 to num_locations - 1. By default the start of
        # a route is node 0.
        routing = pywrapcp.RoutingModel(num_locations, num_vehicles, depot)
        search_parameters = pywrapcp.RoutingModel.DefaultSearchParameters()

        # Setting first solution heuristic: the
        # method for finding a first solution to the problem.
        search_parameters.first_solution_strategy = (
                routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

        search_parameters.time_limit_ms = search_time_limit

        # The 'PATH_CHEAPEST_ARC' method does the following:
        # Starting from a route "start" node, connect it to the node which produces the
        # cheapest route segment, then extend the route by iterating on the last
        # node added to the route.

        # Put callbacks to the distance function and travel time functions here.

        dist_between_locations = CreateDistanceCallback(locations)
        dist_callback = dist_between_locations.Distance

        routing.SetArcCostEvaluatorOfAllVehicles(dist_callback)
        demands_at_locations = CreateDemandCallback(demands)
        demands_callback = demands_at_locations.Demand

        # Adding capacity dimension constraints.
        VehicleCapacity = vehicle_capacity;
        NullCapacitySlack = 0;
        fix_start_cumul_to_zero = True
        capacity = "Capacity"

        routing.AddDimension(demands_callback, NullCapacitySlack, VehicleCapacity,
                                                 fix_start_cumul_to_zero, capacity)
        
        # Solve displays a solution if any.
        assignment = routing.SolveWithParameters(search_parameters)
        if assignment:
            data = create_data_array()
            locations = data[0]
            demands = data[1]
#             start_times = data[2]
            size = len(locations)
            # Solution cost.
            print ("Total distance of all routes: " , str(assignment.ObjectiveValue()))
            # Inspect solution.
            capacity_dimension = routing.GetDimensionOrDie(capacity);
#             time_dimension = routing.GetDimensionOrDie(time);

            for vehicle_nbr in xrange(num_vehicles):
                _output = []
                index = routing.Start(vehicle_nbr)
                plan_output = 'Route {0}:'.format(vehicle_nbr)

                while not routing.IsEnd(index):
                    node_index = routing.IndexToNode(index)
                    load_var = capacity_dimension.CumulVar(index)
#                     time_var = time_dimension.CumulVar(index)
                    plan_output +=                                         " {node_index} Load({load})  -> ".format(
                                                node_index=node_index,
                                                load=assignment.Value(load_var))
                    _output.append(node_index)
                    index = assignment.Value(routing.NextVar(index))

                node_index = routing.IndexToNode(index)
                # _output.append(node_index)
                load_var = capacity_dimension.CumulVar(index)
#                 time_var = time_dimension.CumulVar(index)
                plan_output +=                                     " {node_index} Load({load})".format(
                                            node_index=node_index,
                                            load=assignment.Value(load_var))
                
                _output = _output[1:]
                output.append(_output)
                print (plan_output)
                
                return output
        else:
            print ('No solution found.')
    else:
        print ('Specify an instance greater than 0.')

def main_2(vehicle_capacity = 0, depot_location  = [], customer_locations  = [], customer_demands = []):
    depot_num = len(depot_location)
    
    data_array = np.array_split(customer_locations, depot_num)
    customer_demands = np.array_split(customer_demands, depot_num)
    
    root_output = []
    
    for data_depot_i in data_array:
        locations = data_depot_i
#         locations.append(depot_location.pop())
        np.append(locations, depot_location.pop())
        
        # Create the data.
        data = create_data_array()
        demands = customer_demands.pop()
    #     start_times = data[2]
        num_locations = len(locations)
        depot = 0
        num_vehicles = 5
        search_time_limit = 5000
        output = []

        # Create routing model.
        if num_locations > 0:

            # The number of nodes of the VRP is num_locations.
            # Nodes are indexed from 0 to num_locations - 1. By default the start of
            # a route is node 0.
            routing = pywrapcp.RoutingModel(num_locations, num_vehicles, depot)
            search_parameters = pywrapcp.RoutingModel.DefaultSearchParameters()

            search_parameters.time_limit_ms = search_time_limit

            # Setting first solution heuristic: the
            # method for finding a first solution to the problem.
            search_parameters.first_solution_strategy = (
                    routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

            # The 'PATH_CHEAPEST_ARC' method does the following:
            # Starting from a route "start" node, connect it to the node which produces the
            # cheapest route segment, then extend the route by iterating on the last
            # node added to the route.

            # Put callbacks to the distance function and travel time functions here.

            dist_between_locations = CreateDistanceCallback(locations)
            dist_callback = dist_between_locations.Distance

            routing.SetArcCostEvaluatorOfAllVehicles(dist_callback)
            demands_at_locations = CreateDemandCallback(demands)
            demands_callback = demands_at_locations.Demand

            # Adding capacity dimension constraints.
            VehicleCapacity = vehicle_capacity;
            NullCapacitySlack = 0;
            fix_start_cumul_to_zero = True
            capacity = "Capacity"

            routing.AddDimension(demands_callback, NullCapacitySlack, VehicleCapacity,
                                                     fix_start_cumul_to_zero, capacity)

            # Solve displays a solution if any.
            assignment = routing.SolveWithParameters(search_parameters)
            if assignment:
                data = create_data_array()
                locations = data[0]
                demands = data[1]
    #             start_times = data[2]
                size = len(locations)
                # Solution cost.
                print ("Total distance of all routes: " , str(assignment.ObjectiveValue()))
                # Inspect solution.
                capacity_dimension = routing.GetDimensionOrDie(capacity);
    #             time_dimension = routing.GetDimensionOrDie(time);

                for vehicle_nbr in xrange(num_vehicles):
                    _output = []
                    index = routing.Start(vehicle_nbr)
                    plan_output = 'Route {0}:'.format(vehicle_nbr)

                    while not routing.IsEnd(index):
                        node_index = routing.IndexToNode(index)
                        load_var = capacity_dimension.CumulVar(index)
    #                     time_var = time_dimension.CumulVar(index)
                        plan_output += \
                                            " {node_index} Load({load})  -> ".format(
                                                    node_index=node_index,
                                                    load=assignment.Value(load_var))
                        _output.append(node_index)
                        index = assignment.Value(routing.NextVar(index))

                    node_index = routing.IndexToNode(index)
                    # _output.append(node_index)
                    load_var = capacity_dimension.CumulVar(index)
    #                 time_var = time_dimension.CumulVar(index)
                    plan_output += \
                                        " {node_index} Load({load})".format(
                                                node_index=node_index,
                                                load=assignment.Value(load_var))
                    _output = _output[1:]
                    output.append(_output)
                    print (plan_output)

                    root_output.append(output)
            else:
                print ('No solution found.')
        else:
            print ('Specify an instance greater than 0.')
    return root_output



def create_data_array():

    locations = [[82, 76], [96, 44], [50, 5], [49, 8], [13, 7], [29, 89], [58, 30], [84, 39],
                             [14, 24], [12, 39], [3, 82], [5, 10], [98, 52], [84, 25], [61, 59], [1, 65],
                             [88, 51], [91, 2], [19, 32], [93, 3], [50, 93], [98, 14], [5, 42], [42, 9],
                             [61, 62], [9, 97], [80, 55], [57, 69], [23, 15], [20, 70], [85, 60], [98, 5]]

    demands =    [0, 19, 21, 6, 19, 7, 12, 16, 6, 16, 8, 14, 21, 16, 3, 22, 18,
                         19, 1, 24, 8, 12, 4, 8, 24, 24, 2, 20, 15, 2, 14, 9]

    start_times =    [28842, 50891, 10351, 49370, 22553, 53131, 8908,
                                    56509, 54032, 10883, 60235, 46644, 35674, 30304,
                                    39950, 38297, 36273, 52108, 2333, 48986, 44552,
                                    31869, 38027, 5532, 57458, 51521, 11039, 31063,
                                    38781, 49169, 32833, 7392]

    data = [locations, demands]
    return data



# In[ ]:

class RequireJSON(object):

    def process_request(self, req, resp):
        if not req.client_accepts_json:
            raise falcon.HTTPNotAcceptable(
                'This API only supports responses encoded as JSON.',
                href='http://docs.examples.com/api/json')

        if req.method in ('POST', 'PUT'):
            if 'application/json' not in req.content_type:
                raise falcon.HTTPUnsupportedMediaType(
                    'This API only supports requests encoded as JSON.',
                    href='http://docs.examples.com/api/json')


class JSONTranslator(object):

    def process_request(self, req, resp):
        # req.stream corresponds to the WSGI wsgi.input environ variable,
        # and allows you to read bytes from the request body.
        #
        # See also: PEP 3333
        if req.content_length in (None, 0):
            # Nothing to do
            return

        body = req.stream.read()
        if not body:
            raise falcon.HTTPBadRequest('Empty request body',
                                        'A valid JSON document is required.')

        try:
            req.context['doc'] = json.loads(body.decode('utf-8'))

        except (ValueError, UnicodeDecodeError):
            raise falcon.HTTPError(falcon.HTTP_753,
                                   'Malformed JSON',
                                   'Could not decode the request body. The '
                                   'JSON was incorrect or not encoded as '
                                   'UTF-8.')

    def process_response(self, req, resp, resource):
        if 'result' not in req.context:
            return

        resp.body = json.dumps(req.context['result'])


# In[ ]:

class VRPApi:
    def on_get(self, req, resp):
        resp.body = 'OK'

    def on_post(self, req, resp):
        try:
            print req.context
            data = req.context['doc']
            data = data['problem_data']

            json_data = main(data['vehicle_capacity'], data['depot'], 
                             data['customer_locations'], data['customer_demands'])

            resp.body = json.dumps({"routes": json_data})
            
        except KeyError:
            raise falcon.HTTPBadRequest(
                'Missing thing',
                'A thing must be submitted in the request body.')

class mdcvrp:
    def on_get(self, req, resp):
        resp.body = 'OK'

    def on_post(self, req, resp):
        # try:
        print req.context
        data = req.context['doc']
        data = data['problem_data']

        json_data = main_2(data['vehicle_capacity'], data['depots'], 
                         data['customer_locations'], data['customer_demands'])

        resp.body = json.dumps({"routes": json_data})
            
        # except KeyError:
        #     raise falcon.HTTPBadRequest(
        #         'Missing thing',
        #         'A thing must be submitted in the request body.')

class Index:
    def on_get(self, req, res):
            res.body = 'We dont talk anymore !!!'
            
api = falcon.API(middleware=[
    RequireJSON(),
    JSONTranslator(),
])

api.add_route('/cvrp', VRPApi())
api.add_route('/mdcvrp', mdcvrp())
api.add_route('/', Index())

# if __name__ == '__main__':
#     httpd = simple_server.make_server('127.0.0.1', 8000, api)
#     httpd.serve_forever()
