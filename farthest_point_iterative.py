"""Simple Vehicles Routing Problem."""

from __future__ import print_function
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

from time import sleep 
import math
from itertools import combinations

CAB_CAPACITY = 4


def create_data_model():
    """Stores the data for the problem."""
    data = {}
    data['distance_matrix'] = [[0, 3152, 13853, 10005, 10139, 10803, 22217, 12423, 13091, 15793, 14908, 4601, 26473, 12780, 14268, 12636, 12794, 12494, 14520, 16345, 9152, 10641, 8689, 6945, 27162, 6390, 15982, 8227, 27310, 16123, 25554, 22613, 18108, 18238, 12344, 25373, 11892, 16978, 6253, 6970, 20765, 17012], [5370, 0, 12715, 7685, 7819, 8484, 20382, 12050, 11556, 14069, 12863, 6083, 27954, 14261, 15749, 14118, 14275, 13975, 16001, 10174, 10634, 12122, 10170, 5177, 24917, 4456, 13568, 6134, 25445, 14258, 23689, 20747, 15656, 15380, 10809, 23507, 10357, 16068, 4325, 4892, 21588, 13624], [16342, 11802, 0, 7226, 9085, 7565, 32758, 23479, 25899, 26334, 25449, 17055, 35776, 22071, 23558, 21927, 22085, 23035, 25061, 37144, 19378, 17763, 16955, 12477, 28177, 10312, 16052, 10972, 54396, 24237, 39106, 35833, 28648, 28779, 21842, 38925, 21389, 17142, 10937, 10647, 31306, 27553], [11190, 6650, 7994, 0, 2298, 1250, 35880, 19725, 18460, 29456, 28571, 11903, 38898, 25192, 26680, 25049, 25206, 26156, 28182, 13680, 22499, 20885, 15990, 7325, 23895, 5160, 6600, 5140, 30273, 19085, 28516, 25575, 20483, 20207, 16689, 28335, 16237, 9376, 5785, 5495, 26416, 30675], [11324, 6784, 10314, 2298, 0, 1627, 36014, 18493, 17999, 29590, 28705, 12037, 39032, 25326, 26814, 25182, 25340, 26290, 28316, 11091, 22633, 21019, 16124, 5238, 22743, 5051, 6057, 3182, 31375, 19016, 28447, 25505, 20414, 20138, 16620, 28266, 16168, 8587, 5263, 4269, 26346, 30809], [11988, 7448, 8333, 1250, 1631, 0, 36678, 20523, 19258, 30254, 29369, 12701, 39696, 25990, 27478, 25847, 26004, 26954, 28981, 12586, 23297, 21683, 16788, 6733, 22992, 5958, 5876, 4677, 49211, 19883, 29315, 26373, 21282, 21005, 17488, 29133, 17035, 8845, 6758, 5764, 27214, 31473], [19809, 22596, 32011, 33715, 33849, 34513, 0, 10815, 10505, 8430, 8610, 16974, 19768, 7679, 7267, 7607, 8055, 7557, 6143, 24859, 12646, 14194, 15829, 21624, 42239, 23579, 30014, 24804, 24113, 17069, 17438, 15833, 11846, 11594, 15512, 12351, 15060, 31641, 22870, 30963, 7905, 10152], [11956, 14743, 25928, 27631, 27765, 28430, 10808, 0, 965, 4495, 3111, 9961, 23458, 6561, 7358, 6489, 5683, 3212, 5239, 17006, 5633, 8110, 8816, 13770, 27824, 15726, 22161, 16950, 18610, 11566, 16800, 13527, 6343, 6474, 7659, 16619, 7207, 23788, 15017, 24879, 9001, 4186], [11260, 14047, 26583, 18229, 18197, 19557, 10536, 965, 0, 4223, 2941, 10671, 24114, 7216, 8013, 7144, 6339, 3868, 5894, 16311, 5824, 7313, 9007, 13075, 27128, 15031, 21466, 16255, 18440, 11396, 16631, 13358, 6173, 6304, 6964, 16450, 6511, 23093, 14321, 15193, 8831, 4086], [14641, 17429, 27611, 29315, 29449, 30113, 8421, 4442, 3986, 0, 2085, 12574, 25142, 8244, 9042, 8172, 7367, 4889, 6916, 18295, 8246, 9794, 11429, 15059, 35715, 26633, 23450, 18239, 17589, 10545, 15779, 12506, 5322, 5070, 8948, 15598, 8495, 25077, 16305, 26563, 7980, 3628], [12301, 15089, 27524, 19270, 19238, 20599, 9125, 3571, 3260, 2833, 0, 11713, 25055, 8157, 8955, 8085, 7280, 4802, 6828, 17352, 8159, 9706, 11342, 14116, 33948, 16072, 22507, 17296, 15822, 8778, 14013, 10740, 3555, 3521, 8005, 13832, 7552, 24134, 15363, 16234, 6213, 1156], [2509, 5296, 15997, 12149, 12283, 12947, 18782, 8988, 9989, 12358, 11472, 0, 23037, 9344, 10832, 9200, 9358, 9058, 11084, 15168, 5716, 7577, 4228, 10626, 25985, 8534, 18127, 10372, 26939, 14946, 25129, 21856, 14672, 14803, 10040, 24948, 9588, 19123, 8397, 9114, 17330, 13577], [24845, 27632, 35371, 37075, 37209, 37873, 19791, 22483, 24903, 25339, 24454, 22778, 0, 16389, 15218, 16317, 17228, 22039, 19322, 34546, 18382, 18748, 21633, 30004, 45364, 34392, 43052, 35361, 39920, 32876, 38111, 34837, 27653, 27784, 28094, 28530, 27642, 44112, 35017, 34322, 24900, 26558], [12887, 15675, 22798, 24502, 24636, 25300, 8115, 6283, 8086, 8522, 7637, 10820, 16384, 0, 1731, 171, 1184, 3533, 4503, 22589, 6425, 6175, 9675, 18047, 41229, 21819, 30479, 22788, 23103, 16059, 21294, 18021, 10836, 10967, 14499, 21112, 14047, 31539, 22444, 21750, 13494, 9741], [14161, 16948, 24687, 26391, 26525, 27189, 7703, 7081, 8884, 9319, 8434, 12094, 15193, 1731, 0, 1659, 2570, 4441, 4235, 23862, 7698, 8064, 10949, 19320, 42027, 23708, 32368, 24678, 23901, 16857, 22091, 18818, 11633, 11764, 15296, 18452, 14844, 33429, 24333, 23639, 14291, 10538], [12744, 15531, 22654, 24358, 24492, 25156, 8043, 6211, 8015, 8450, 7565, 10677, 16312, 171, 1659, 0, 1040, 3461, 4431, 22445, 6281, 6031, 9532, 17903, 41157, 21675, 30335, 22645, 23031, 15987, 21222, 17949, 10764, 10895, 14427, 21041, 13975, 31396, 22300, 21606, 13422, 9669], [12901, 15689, 22812, 24516, 24650, 25314, 8151, 5402, 7205, 7641, 6756, 10834, 17501, 1184, 2661, 1040, 0, 2763, 3732, 22603, 6439, 6189, 9689, 18061, 40348, 21833, 30493, 22802, 22222, 15178, 20413, 17139, 9955, 10086, 13618, 20231, 13166, 31553, 22458, 21764, 12613, 8860], [11482, 14269, 24452, 26156, 26290, 26954, 7559, 3357, 3870, 5166, 4281, 9415, 21983, 3652, 4450, 3580, 2775, 0, 2024, 18965, 5087, 6635, 8270, 15467, 37873, 23473, 32134, 24443, 19747, 12703, 17938, 14665, 7480, 7611, 9745, 17757, 9293, 26357, 24099, 23404, 10138, 6385], [13512, 16300, 26483, 28186, 28320, 28984, 6582, 5387, 5900, 5239, 5419, 11445, 19738, 4333, 4373, 4261, 3735, 2028, 0, 21668, 7117, 8665, 10300, 17497, 39904, 25504, 34164, 26473, 21778, 14734, 19968, 16695, 9511, 9642, 12321, 15876, 11869, 28387, 26129, 25434, 9245, 6961], [15518, 11323, 35353, 13530, 11077, 12437, 28491, 16364, 15870, 20036, 21358, 15612, 35194, 21501, 22441, 21358, 21515, 18296, 20322, 0, 17874, 19362, 19329, 7532, 12451, 9248, 25136, 9674, 22459, 13942, 24346, 22091, 16708, 16964, 12892, 23192, 12440, 10898, 8936, 9030, 21272, 19465], [7506, 10293, 20950, 22653, 22787, 23452, 15417, 6137, 8557, 8993, 8107, 5439, 18480, 4787, 6275, 4643, 4801, 5693, 7719, 17207, 0, 1663, 4294, 12665, 28025, 19971, 21056, 20940, 23574, 16530, 21765, 18491, 11307, 11438, 10536, 21583, 10084, 23556, 13394, 19901, 13965, 10212], [8761, 11548, 19063, 20767, 20900, 21565, 15679, 6399, 8820, 9255, 8370, 6694, 19409, 5704, 7191, 5560, 5718, 5955, 7981, 18462, 1663, 0, 3523, 13920, 29280, 18084, 26744, 19053, 23836, 16792, 22027, 18754, 11569, 11700, 12010, 21846, 11558, 27804, 18709, 18014, 14227, 10474], [6766, 9553, 16302, 18774, 18908, 19573, 18434, 10246, 11574, 12010, 11124, 4246, 22689, 8996, 10484, 8852, 9010, 8710, 10736, 18667, 5368, 3523, 0, 14125, 29485, 12791, 22516, 14629, 26591, 15904, 24781, 21508, 14324, 14455, 11996, 24600, 11544, 25015, 12654, 13371, 16982, 13229], [8698, 4158, 12604, 7792, 5338, 6699, 21824, 13492, 12998, 20109, 14306, 9411, 30422, 16729, 18217, 16585, 16743, 15424, 17450, 7561, 13101, 14590, 13498, 0, 14124, 2114, 8607, 3396, 25202, 14015, 23446, 20504, 15413, 15137, 11619, 23265, 11167, 11107, 1405, 2550, 21346, 15067], [26474, 17619, 26845, 28650, 20974, 21162, 41889, 27320, 26825, 33434, 34756, 26568, 46150, 32457, 41713, 32313, 40038, 37560, 39586, 13056, 28829, 30318, 30284, 14129, 0, 14691, 16629, 13499, 33150, 24897, 30958, 28703, 32116, 32247, 23848, 37033, 23396, 8723, 15017, 14587, 35114, 32863], [7469, 2928, 10186, 5156, 5133, 6553, 25797, 14929, 14435, 19373, 18488, 8181, 30053, 22654, 24141, 22510, 22667, 16073, 18100, 9288, 12732, 18346, 12269, 2114, 14828, 0, 8958, 1816, 26247, 15060, 24491, 21549, 16458, 16182, 12664, 24310, 12212, 11811, 989, 782, 22390, 16504], [17170, 12629, 16548, 6600, 5912, 5859, 30224, 21892, 21397, 47617, 22705, 17882, 44919, 31213, 32701, 31069, 31227, 32177, 34203, 27238, 28520, 26905, 21969, 8636, 18272, 9795, 0, 7925, 44490, 22414, 49317, 42886, 23813, 23536, 20019, 31664, 19566, 5053, 10007, 9013, 29745, 23466], [9539, 4998, 11142, 5140, 3254, 4675, 34297, 16620, 16126, 27873, 26988, 10251, 37315, 23610, 25097, 23466, 23623, 24573, 26600, 9875, 20916, 19302, 14338, 3364, 13555, 1823, 7481, 0, 30158, 17142, 26574, 23632, 18541, 18264, 14747, 26392, 14295, 9010, 2149, 1409, 24473, 29092], [23777, 24522, 52994, 54798, 30935, 32295, 24331, 19555, 19099, 15876, 17198, 27687, 40255, 23357, 24155, 23285, 22480, 20002, 22028, 21875, 23359, 24907, 26542, 23606, 31763, 25562, 42778, 29532, 0, 9304, 6688, 7416, 14558, 14689, 16036, 14280, 15584, 43592, 24853, 30620, 16226, 15305], [14782, 15526, 24795, 19765, 19733, 21094, 17803, 13027, 12571, 9349, 10670, 14876, 33727, 16829, 17627, 16757, 15952, 13474, 15501, 15672, 16831, 18379, 16307, 14611, 27500, 16567, 23002, 17791, 9366, 0, 8914, 5638, 5011, 5266, 6191, 12504, 5739, 22454, 15858, 16729, 10585, 8777], [24398, 25142, 53495, 29381, 32745, 34106, 16634, 17478, 17022, 13799, 15121, 25610, 38178, 21280, 22078, 21208, 20403, 17925, 19951, 23686, 21282, 22829, 24465, 24227, 30593, 26183, 43278, 31343, 6706, 8829, 0, 1817, 12481, 12612, 15807, 10723, 15355, 38629, 25474, 26345, 8803, 13228], [21126, 21871, 31139, 26109, 26078, 27438, 15039, 14207, 13751, 10528, 11850, 22339, 34906, 18009, 18806, 17937, 17131, 14654, 16680, 22083, 18011, 19558, 21194, 20956, 28991, 22912, 41676, 24136, 7431, 5654, 1817, 0, 9210, 9341, 12536, 9127, 12084, 37027, 22202, 23074, 7208, 9957], [17240, 20027, 30210, 19974, 19942, 21303, 11817, 7041, 6585, 3362, 4684, 15173, 27740, 10843, 11640, 10771, 9965, 7488, 9514, 17907, 10845, 12392, 14028, 14820, 32339, 16776, 23211, 18000, 14213, 5011, 12404, 9130, 0, 446, 5732, 12222, 5419, 24689, 16067, 16938, 5648, 2368], [14675, 15459, 30341, 19698, 19666, 21026, 11948, 7172, 6716, 3493, 3525, 15304, 27871, 10974, 11771, 10902, 10096, 7619, 9645, 17774, 10976, 12523, 14159, 14544, 32470, 16500, 22934, 17724, 14344, 5266, 12534, 9261, 446, 0, 5456, 12353, 5143, 24556, 15790, 16662, 5779, 2092], [10368, 13156, 21566, 16537, 16505, 17865, 15218, 7372, 6669, 12182, 7699, 9780, 27821, 14128, 14003, 13985, 12329, 9858, 11884, 13335, 10501, 11989, 11956, 11383, 24152, 13339, 19773, 14563, 17275, 6088, 15519, 12578, 5769, 5493, 0, 15338, 499, 20117, 12629, 13501, 13419, 6437], [23809, 24553, 33822, 28792, 28760, 30121, 12341, 16893, 16437, 13215, 14536, 23903, 28499, 20695, 21493, 20623, 19818, 17340, 19367, 24699, 20697, 22245, 23880, 23638, 37303, 25594, 32029, 26818, 14313, 12133, 8632, 10179, 11896, 12027, 15218, 0, 14766, 31481, 24885, 25756, 5743, 12643], [9916, 12703, 21114, 16084, 16053, 17413, 14766, 6920, 6217, 11730, 7247, 9327, 27369, 13676, 13551, 13532, 11876, 9406, 11432, 12882, 10048, 11537, 11504, 10931, 23700, 12886, 19321, 14111, 16823, 5636, 15067, 12125, 5420, 5144, 462, 14886, 0, 19665, 12177, 13049, 12967, 6004], [18434, 13893, 17328, 8996, 8275, 8463, 31066, 22734, 22240, 28038, 23548, 19146, 46210, 32504, 33992, 32361, 32518, 33468, 35495, 11065, 29811, 28197, 23233, 10182, 8393, 11086, 5243, 9217, 45270, 21944, 50097, 36409, 24710, 23634, 20116, 31194, 19664, 0, 10914, 10304, 29274, 27467], [8167, 3626, 10811, 5782, 5335, 6756, 23429, 15097, 14603, 21322, 15910, 8879, 30751, 17058, 18545, 16914, 17072, 16772, 18798, 8966, 13430, 18971, 12967, 1405, 15161, 989, 9290, 2148, 26415, 15228, 24659, 21717, 16626, 16350, 12832, 24477, 12380, 12144, 0, 1425, 22558, 16671], [7804, 3263, 9901, 4871, 4351, 5771, 33056, 15365, 14871, 26632, 25747, 8516, 36074, 22368, 23856, 22225, 22382, 23332, 25359, 10971, 19675, 18061, 12604, 2550, 14651, 782, 8577, 1356, 26683, 15496, 24927, 21985, 16894, 16618, 13100, 24746, 12648, 10107, 1425, 0, 22826, 27851], [19796, 22692, 32766, 26931, 26899, 28260, 8210, 9597, 9141, 5918, 6595, 17729, 25182, 13399, 12681, 13327, 12521, 10044, 9505, 22838, 13401, 14948, 16584, 21777, 35438, 23733, 30168, 24957, 17312, 10268, 9912, 8307, 5671, 5802, 13358, 5746, 12905, 29621, 23024, 23896, 0, 6253], [13060, 15847, 28504, 20029, 19997, 21357, 10104, 4350, 4039, 2538, 1155, 12471, 26034, 9137, 9934, 9065, 8259, 5782, 7808, 20636, 9139, 10686, 12322, 14875, 33226, 16831, 23266, 18055, 15100, 8056, 13291, 10018, 2368, 2092, 6400, 13110, 6018, 27419, 16121, 16993, 6242, 0]]
    # data['num_vehicles'] = 2
    # data['starts'] = [1, 3]
    # data['ends'] = [0, 0]
    return data

list_of_nodes_visited_during_best_path = []
def print_solution(data, manager, routing, solution):
    """Prints solution on console."""
    min_route_distance = 999999999
    for vehicle_id in range(data['num_vehicles']):
        index = routing.Start(vehicle_id)
        plan_output = 'Route for vehicle {}:\n'.format(vehicle_id)
        route_distance = 0
        temp_list_of_nodes_visited = []
        while not routing.IsEnd(index):
            plan_output += ' {} -> '.format(manager.IndexToNode(index))
            temp_list_of_nodes_visited.append(manager.IndexToNode(index))
            previous_index = index
            index = solution.Value(routing.NextVar(index))
            route_distance += routing.GetArcCostForVehicle(
                previous_index, index, vehicle_id)
        if route_distance < min_route_distance:
            list_of_nodes_visited_during_best_path.clear()
            list_of_nodes_visited_during_best_path.append(temp_list_of_nodes_visited)
        plan_output += '{}\n'.format(manager.IndexToNode(index))
        plan_output += 'Distance of the route: {}m\n'.format(route_distance)
        #print(plan_output)
        min_route_distance = min(route_distance, min_route_distance)
    print('Minimum of the route distances: {}m'.format(min_route_distance))
    print(list_of_nodes_visited_during_best_path)

def delete_the_visited_nodes_from_the_distance_matrix():
    pass


def get_the_farthest_point(distance_matrix):
    print(distance_matrix[0].index(max(distance_matrix[0])))
    print(max(distance_matrix[0]))
    return distance_matrix[0].index(max(distance_matrix[0]))

def constraint_solver(data,no_of_cabs,no_of_employees):
    farthest_point_from_depot = get_the_farthest_point(data['distance_matrix'])
    data['starts'] = [farthest_point_from_depot]*no_of_cabs
    data['ends'] = [0]*no_of_cabs
    data['num_vehicles'] = no_of_cabs
    data['demands'] = [1]*no_of_employees
    data['demands'].insert(0,0)
    data['vehicle_capacities'] = [CAB_CAPACITY] * no_of_cabs 
    print("data-----------------------")
    print(data['num_vehicles'])


    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(len(data['distance_matrix']),
                                        data['num_vehicles'], data['starts'],
                                        data['ends'])

    # Create Routing Model.
    routing = pywrapcp.RoutingModel(manager)
    #print(routing)


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

    # # Add Distance constraint.
    # dimension_name = 'Distance'
    # routing.AddDimension(
    #     transit_callback_index,
    #     0,  # no slack
    #     50000,  # vehicle maximum travel distance
    #     True,  # start cumul to zero
    #     dimension_name)
    # distance_dimension = routing.GetDimensionOrDie(dimension_name)
    # distance_dimension.SetGlobalSpanCostCoefficient(100)


    def demand_callback(from_index):
        """Returns the demand of the node."""
        # Convert from routing variable Index to demands NodeIndex.
        from_node = manager.IndexToNode(from_index)
        return data['demands'][from_node]

    demand_callback_index = routing.RegisterUnaryTransitCallback(
        demand_callback)
    routing.AddDimensionWithVehicleCapacity(
        demand_callback_index,
        0,  # null capacity slack
        data['vehicle_capacities'],  # vehicle maximum capacities
        True,  # start cumul to zero
        'Capacity')


    # Setting first solution heuristic.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.time_limit.seconds = 5
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

    return search_parameters, manager, routing


def main():
    """Entry point of the program."""
    # Instantiate the data problem.
    data = create_data_model()
    no_of_employees = len(data['distance_matrix']) - 1

    no_of_cabs = math.ceil(no_of_employees / CAB_CAPACITY)

    search_parameters, manager, routing = constraint_solver(data,no_of_cabs,no_of_employees)

    # farthest_point_from_depot = get_the_farthest_point(data['distance_matrix'])
    # data['starts'] = [farthest_point_from_depot]*no_of_cabs
    # data['ends'] = [0]*no_of_cabs
    # data['num_vehicles'] = no_of_cabs
    # data['demands'] = [1]*no_of_employees
    # data['demands'].insert(0,0)
    # data['vehicle_capacities'] = [CAB_CAPACITY] * no_of_cabs 
    # print("data-----------------------")
    # print(data)


    # # Create the routing index manager.
    # manager = pywrapcp.RoutingIndexManager(len(data['distance_matrix']),
    #                                     data['num_vehicles'], data['starts'],
    #                                     data['ends'])

    # # Create Routing Model.
    # routing = pywrapcp.RoutingModel(manager)
    # #print(routing)


    # # Create and register a transit callback.
    # def distance_callback(from_index, to_index):
    #     """Returns the distance between the two nodes."""
    #     # Convert from routing variable Index to distance matrix NodeIndex.
    #     from_node = manager.IndexToNode(from_index)
    #     to_node = manager.IndexToNode(to_index)
    #     return data['distance_matrix'][from_node][to_node]

    # transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    # # Define cost of each arc.
    # routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # # # Add Distance constraint.
    # # dimension_name = 'Distance'
    # # routing.AddDimension(
    # #     transit_callback_index,
    # #     0,  # no slack
    # #     10000000,  # vehicle maximum travel distance
    # #     True,  # start cumul to zero
    # #     dimension_name)
    # # distance_dimension = routing.GetDimensionOrDie(dimension_name)
    # # distance_dimension.SetGlobalSpanCostCoefficient(100)


    # def demand_callback(from_index):
    #     """Returns the demand of the node."""
    #     # Convert from routing variable Index to demands NodeIndex.
    #     from_node = manager.IndexToNode(from_index)
    #     return data['demands'][from_node]

    # demand_callback_index = routing.RegisterUnaryTransitCallback(
    #     demand_callback)
    # routing.AddDimensionWithVehicleCapacity(
    #     demand_callback_index,
    #     0,  # null capacity slack
    #     data['vehicle_capacities'],  # vehicle maximum capacities
    #     True,  # start cumul to zero
    #     'Capacity')


    # # Setting first solution heuristic.
    # search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    # search_parameters.time_limit.seconds = 10
    # search_parameters.first_solution_strategy = (
    #     routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

    # Solve the problem.
    solution = routing.SolveWithParameters(search_parameters)
    print("solution", solution)
    new_cab_capacity = no_of_cabs + 1
    while not solution:
        print("from while loop", solution)
        search_parameters, manager, routing = constraint_solver(data,new_cab_capacity,no_of_employees)
        solution = routing.SolveWithParameters(search_parameters)
        new_cab_capacity+=1


    # Print solution on console.
    if solution:
        print_solution(data, manager, routing, solution)


if __name__ == '__main__':
    main()