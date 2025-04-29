import os
import h3
import shapely as shp
from h3 import geo_to_h3shape
from shapely.geometry.polygon import Polygon
from shapely.geometry.multipolygon import MultiPolygon
import geopandas as gpd
import matplotlib.pyplot as plt
import numpy as np
from typing import List, Dict
import gurobipy as gp
from gurobipy import GRB
from gurobipy import quicksum

# Ann Arbor police department (lat, long).
ANN_ARBOR_PD_LAT_LNG = [42.28196926076756, -83.74584246097976]
TRADER_JOES_LAT_LNG = [42.258746816570365, -83.71303193019678]
MULTI_LAT_LNG = [
    [42.27419348603703, -83.75334117492699],
    [42.272288185548014, -83.75583026469232],
    [42.26790577584151, -83.75342700560856],
    [42.25119901449918, -83.70922420460363],
    [42.246497454817295, -83.71797893412305],
    [42.2548202472297, -83.71471736822365],
    [42.29743351411518, -83.7245878966034],
    [42.29590984050696, -83.72862193863685],
    [42.3072095425411, -83.72810695454747]
]

# Resolution for H3 cells (0-15).
H3_RESOLUTION = 7

# Type alias for string representation of H3Index.
H3IndexCell = str
H3IndexEdge = str

class SurveillanceProgramParams:
    def __init__(self,
                 map_cells: List[H3IndexCell],
                 num_drones: int,
                 drone_recharge_time: float,
                 drone_range: float,
                 drone_speed: float) -> None:
        """"""
        # List of nodes in the graph, where node_indices[node_num] = node_index.
        self.node_indices: List[H3IndexCell] = map_cells

        # List of nodes in the graph which must be viewed by drones. Drone = node_index.
        # self.drone_node_numbers: List[int] = [12, 13, 3, 9]
        # self.drone_node_numbers: List[int] = [4]  #

        # Single cover (Trader Joe's)
        # single_node_index = h3.latlng_to_cell(TRADER_JOES_LAT_LNG[0],
        #                                       TRADER_JOES_LAT_LNG[1],
        #                                       H3_RESOLUTION)
        # single_node_number = [number for number, index in enumerate(self.node_indices) if index == single_node_index][0]
        # self.drone_node_numbers: List[int] = [single_node_number]

        # Multi cover.
        # multi_node_indices = [h3.latlng_to_cell(lat_lng[0], lat_lng[1], H3_RESOLUTION) for lat_lng in MULTI_LAT_LNG]
        # multi_node_numbers_set = {number for number, index in enumerate(self.node_indices) if index in multi_node_indices}
        # self.drone_node_numbers = list(multi_node_numbers_set)

        # Full cover
        self.drone_node_numbers: List[int] = [i for i in range(len(self.node_indices))]

        # The index of the node where all drones must start and finish their circuits.
        self.base_node_index: H3IndexCell = h3.latlng_to_cell(ANN_ARBOR_PD_LAT_LNG[0],
                                                              ANN_ARBOR_PD_LAT_LNG[1],
                                                              H3_RESOLUTION)
        # self.base_node_index: H3IndexCell = h3.latlng_to_cell(MY_HOUSE_LAT_LNG[0],
        #                                                       MY_HOUSE_LAT_LNG[1],
        #                                                       H3_RESOLUTION)
        # self.base_node_index: H3IndexCell = h3.latlng_to_cell(TRADER_JOES_LAT_LNG[0],
        #                                                       TRADER_JOES_LAT_LNG[1],
        #                                                       H3_RESOLUTION)

        # The number of the node where all drones must start and finish their circuits.
        self.base_node_number: int = [number for number, index in enumerate(self.node_indices) \
                                      if index == self.base_node_index][0]

        # List of arcs in the graph, where arc_indices[arc_num] = arc_index.
        # self.arc_indices: List[H3IndexEdge] = [h3.cells_to_directed_edge(origin, destination) \
        #                                        for origin in map_cells \
        #                                        for destination in map_cells \
        #                                        if h3.are_neighbor_cells(origin, destination)]
        self.arc_indices: List[H3IndexEdge] = []
        for origin in map_cells:
            for destination in map_cells:
                if h3.grid_distance(origin, destination) == 1:
                    self.arc_indices.append(h3.cells_to_directed_edge(origin, destination))

        # List of arc lengths in the graph, where arc_lengths_km[arc_num] = arc_length_km.
        self.arc_lengths_km: List[float] = [h3.edge_length(arc_index, unit='km') for arc_index in self.arc_indices]

        # Dictionary mapping node number to a list of the arc numbers that flow out of node node_number.
        self.node_number_to_outflow_arc_numbers: Dict[int, List[int]] = { \
            node_number : [arc_number for arc_number, arc_index in enumerate(self.arc_indices) \
                           if h3.get_directed_edge_origin(arc_index) == node_index] \
            for node_number, node_index in enumerate(self.node_indices)}

        # Dictionary mapping node number to a list of the arc numbers that flow into node node_number.
        self.node_number_to_inflow_arc_numbers: Dict[int, List[int]] = { \
            node_number : [arc_number for arc_number, arc_index in enumerate(self.arc_indices) \
                           if h3.get_directed_edge_destination(arc_index) == node_index] \
            for node_number, node_index in enumerate(self.node_indices)}

        # List of drone numbers.
        self.drone_numbers: List[int] = [i for i in range(num_drones)]

        # Speed the drones in kph.
        self.drone_speed_kph: float = drone_speed

        # Range of the drones in km.
        self.drone_range_km: float = drone_range

        # Recharge time of the drones in hours.
        self.drone_recharge_time_h: float = drone_recharge_time


# Functioning solver with subtour problem.
# class MinTimeTotalCoverageSingleCircuitProgram:
#     def __init__(self, params: SurveillanceProgramParams) -> None:
#         """"""
#         self.program = gp.Model("MinTimeTotalCoverageSingleCircuitProgram")
#
#         # Add decision variable drone_flows where drone_flows[drone_number][arc_number] is the flow of drone
#         # drone_number across arc arc_number.
#         self.drone_flows = self.program.addVars(len(params.drone_numbers), len(params.arc_indices),
#                                                 lb=0,
#                                                 vtype=GRB.INTEGER)
#
#         # Add continuous decision variable to represent the max circuit time for any single drone.
#         self.max_drone_circuit_time_h = self.program.addVar(vtype=GRB.CONTINUOUS,
#                                                             name="max_drone_circuit_time_h")
#
#         # Minimize the max circuit time for any single drone.
#         self.program.setObjective(self.max_drone_circuit_time_h, GRB.MINIMIZE)
#
#         # Upper bound the circuit time of each drone with the max circuit time variable.
#         self.program.addConstrs(1/params.drone_speed_kph * \
#                                  quicksum([self.drone_flows[drone_number, arc_number] \
#                                            for arc_number in range(len(params.arc_indices))]) \
#                                  <= self.max_drone_circuit_time_h for drone_number in params.drone_numbers)
#
#         # Add coverage constraint for all nodes that must be observed by a drone.
#         self.program.addConstrs(quicksum([self.drone_flows[drone_number, arc_number] \
#                                      for drone_number in params.drone_numbers \
#                                      for arc_number in params.node_number_to_inflow_arc_numbers[node_number]]) >= 1 \
#                                     for node_number in params.drone_node_numbers)
#
#         # Add flow constraints for all drones for all nodes in the graph.
#         self.program.addConstrs(quicksum([self.drone_flows[drone_number, out_arc_number] \
#                                      for out_arc_number in params.node_number_to_outflow_arc_numbers[node_number]]) - \
#                                 quicksum([self.drone_flows[drone_number, in_arc_number] \
#                                      for in_arc_number in params.node_number_to_inflow_arc_numbers[node_number]]) == 0 \
#                                 for drone_number in params.drone_numbers \
#                                 for node_number in range(len(params.node_indices)))
#
#         # Add single circuit constraint for each drone.
#         self.program.addConstrs(quicksum([self.drone_flows[drone_number, arc_number] \
#                                      for arc_number in params.node_number_to_outflow_arc_numbers[params.base_node_number]]) <= 1 \
#                                 for drone_number in params.drone_numbers)
#
#         # Add range constraint for each drone.
#         self.program.addConstrs(quicksum([self.drone_flows[drone_number, arc_number] * params.arc_lengths_km[arc_number] \
#                                       for arc_number in range(len(params.arc_indices))]) \
#                                  <= params.drone_range_km \
#                                  for drone_number in params.drone_numbers)
#
#     def solve(self) -> None:
#         """"""
#         self.program.optimize()

# MTZ formulation which is infeasible due to peninsulas.
# class MinTimeTotalCoverageSingleCircuitProgram:
#     def __init__(self, params: SurveillanceProgramParams) -> None:
#         """"""
#         self.program = gp.Model("MinTimeTotalCoverageSingleCircuitProgram")
#
#         # Add decision variable drone_flows where drone_flows[drone_number][arc_number] is the flow of drone
#         # drone_number across arc arc_number.
#         self.drone_flows = self.program.addVars(len(params.drone_numbers), len(params.arc_indices),
#                                                 lb=0,
#                                                 vtype=GRB.INTEGER)
#
#         # MTZ variables.
#         # Add continuous decision variables to represent ordering for each node other than the starting node.
#         self.ordering_variables = self.program.addVars(len(params.drone_numbers), len(params.node_indices)-1,
#                                                        lb=0,
#                                                        vtype=GRB.CONTINUOUS)
#
#         # Add continuous decision variable to represent the max circuit time for any single drone.
#         self.max_drone_circuit_time_h = self.program.addVar(vtype=GRB.CONTINUOUS,
#                                                             name="max_drone_circuit_time_h")
#
#         # Minimize the max circuit time for any single drone.
#         self.program.setObjective(self.max_drone_circuit_time_h, GRB.MINIMIZE)
#
#         # Upper bound the circuit time of each drone with the max circuit time variable.
#         self.program.addConstrs(1/params.drone_speed_kph * \
#                                  quicksum([self.drone_flows[drone_number, arc_number] \
#                                            for arc_number in range(len(params.arc_indices))]) \
#                                  <= self.max_drone_circuit_time_h for drone_number in params.drone_numbers)
#
#         # Add coverage constraint for all nodes that must be observed by a drone.
#         self.program.addConstrs(quicksum([self.drone_flows[drone_number, arc_number] \
#                                      for drone_number in params.drone_numbers \
#                                      for arc_number in params.node_number_to_inflow_arc_numbers[node_number]]) >= 1 \
#                                     for node_number in params.drone_node_numbers)
#
#         # Add flow constraints for all drones for all nodes in the graph.
#         self.program.addConstrs(quicksum([self.drone_flows[drone_number, out_arc_number] \
#                                      for out_arc_number in params.node_number_to_outflow_arc_numbers[node_number]]) - \
#                                 quicksum([self.drone_flows[drone_number, in_arc_number] \
#                                      for in_arc_number in params.node_number_to_inflow_arc_numbers[node_number]]) == 0 \
#                                 for drone_number in params.drone_numbers \
#                                 for node_number in range(len(params.node_indices)))
#
#         # MTZ constraint.
#         # Add ordering constraints for all edges except those entering and exiting the start node to remove sub-tours
#         # for each drone.
#         self.program.addConstrs(self.ordering_variables[drone_number, [ \
#                                     node_number for node_number, node_index in enumerate(params.node_indices) \
#                                     if node_index == h3.get_directed_edge_origin(arc_index)][0]] - \
#                                 self.ordering_variables[drone_number, [ \
#                                     node_number for node_number, node_index in enumerate(params.node_indices) \
#                                     if node_index == h3.get_directed_edge_destination(arc_index)][0]] + \
#                                 self.drone_flows[drone_number, arc_number] <= 0 \
#                                 for arc_number, arc_index in enumerate(params.arc_indices) \
#                                 for drone_number in range(len(params.drone_numbers)) if \
#                                 h3.get_directed_edge_origin(arc_index) != params.base_node_index and \
#                                 h3.get_directed_edge_destination(arc_index) != params.base_node_index)
#
#         # Add single circuit constraint for each drone.
#         self.program.addConstrs(quicksum([self.drone_flows[drone_number, arc_number] \
#                                      for arc_number in params.node_number_to_outflow_arc_numbers[params.base_node_number]]) <= 1 \
#                                 for drone_number in params.drone_numbers)
#
#         # Add range constraint for each drone.
#         self.program.addConstrs(quicksum([self.drone_flows[drone_number, arc_number] * params.arc_lengths_km[arc_number] \
#                                       for arc_number in range(len(params.arc_indices))]) \
#                                  <= params.drone_range_km \
#                                  for drone_number in params.drone_numbers)
#
#     def solve(self) -> None:
#         """"""
#         self.program.optimize()


class MinTimeTotalCoverageSingleCircuitProgram:
    def __init__(self, params: SurveillanceProgramParams) -> None:
        """"""
        self.program = gp.Model("MinTimeTotalCoverageSingleCircuitProgram")

        # Add decision variable drone_flows where drone_flows[drone_number][arc_number] is the flow of drone
        # drone_number across arc arc_number.
        self.drone_flows = self.program.addVars(len(params.drone_numbers), len(params.arc_indices),
                                                lb=0,
                                                vtype=GRB.INTEGER)

        # Artificial flow variables.
        # Add continuous decision variables for artificial flow to force over drone flows as a way of eliminating the
        # sub-tour problem.
        self.ordering_flows = self.program.addVars(len(params.drone_numbers), len(params.arc_indices),
                                                   lb=0,
                                                   vtype=GRB.CONTINUOUS)

        # Coverage selection variables.
        # Add binary variables for each drone for each node in N_d \ i_base t, which are 1 if the drone deposits flow,
        # and zero otherwise.
        num_drone_nodes_not_base = len(params.drone_node_numbers) \
            if params.base_node_number not in params.drone_node_numbers \
            else len(params.drone_node_numbers)-1
        drone_node_numbers_not_base = [node_number for node_number in params.drone_node_numbers if node_number != params.base_node_number]
        # drone_node_numbers_not_base = []
        # offset = 0
        # for i, node_number in enumerate(params.drone_node_numbers):
        #     if node_number != params.base_node_number:
        #         drone_node_numbers_not_base.append(node_number-offset)
        #     else:
        #         # Don't add base node number, but offset all nodes after this so they correspond to the correct place.
        #         offset = 1
        self.coverage_selection = self.program.addVars(len(params.drone_numbers), num_drone_nodes_not_base,
                                                       vtype=GRB.BINARY)

        # Add continuous decision variable to represent the max circuit time for any single drone.
        self.max_drone_circuit_time_h = self.program.addVar(vtype=GRB.CONTINUOUS,
                                                            name="max_drone_circuit_time_h")

        # Minimize the max circuit time for any single drone.
        self.program.setObjective(self.max_drone_circuit_time_h, GRB.MINIMIZE)

        # Upper bound the circuit time of each drone with the max circuit time variable.
        self.program.addConstrs(1/params.drone_speed_kph * \
                                 quicksum([self.drone_flows[drone_number, arc_number] * \
                                           params.arc_lengths_km[arc_number] \
                                           for arc_number in range(len(params.arc_indices))]) \
                                 <= self.max_drone_circuit_time_h for drone_number in params.drone_numbers)

        # Add flow constraints for all drones for all nodes in the graph.
        self.program.addConstrs(quicksum([self.drone_flows[drone_number, out_arc_number] \
                                     for out_arc_number in params.node_number_to_outflow_arc_numbers[node_number]]) - \
                                quicksum([self.drone_flows[drone_number, in_arc_number] \
                                     for in_arc_number in params.node_number_to_inflow_arc_numbers[node_number]]) == 0 \
                                for drone_number in params.drone_numbers \
                                for node_number in range(len(params.node_indices)))

        # Artificial flow constraint.
        # Force enough artificial flow out of the base node for each node in N_d to \ i_base consume 1 unit of flow.
        # self.program.addConstrs(quicksum([self.ordering_flows[drone_number, arc_number] \
        #                                   for arc_number in params.node_number_to_outflow_arc_numbers[params.base_node_number]]) - \
        #                         quicksum([self.ordering_flows[drone_number, arc_number] \
        #                                   for arc_number in params.node_number_to_inflow_arc_numbers[params.base_node_number]]) \
        #                         == num_drone_nodes_not_base for drone_number in params.drone_numbers)

        # Artificial flow constraint.
        # Force enough artificial flow out of the base node for each node in N_d \ i_base to consume 1 unit of flow.
        self.program.addConstr(quicksum([quicksum([self.ordering_flows[drone_number, arc_number] \
                                         for arc_number in params.node_number_to_outflow_arc_numbers[params.base_node_number]]) -
                               quicksum([self.ordering_flows[drone_number, arc_number] \
                                         for arc_number in params.node_number_to_inflow_arc_numbers[params.base_node_number]]) \
                               for drone_number in params.drone_numbers]) == num_drone_nodes_not_base)

        # # Artificial flow constraint.
        # # Force each node in N_d \ i_base to consume 1 unit of artificial flow.
        # self.program.addConstrs(quicksum([quicksum([self.ordering_flows[drone_number, arc_number] \
        #                                             for arc_number in params.node_number_to_outflow_arc_numbers[node_number]]) - \
        #                                   quicksum([self.ordering_flows[drone_number, arc_number] \
        #                                             for arc_number in params.node_number_to_inflow_arc_numbers[node_number]]) \
        #                                   for drone_number in params.drone_numbers]) == -1 \
        #                         for node_number in params.drone_node_numbers if node_number != params.base_node_number)

        # Artificial flow constraint.
        # Force artificial inflow = artificial outflow for all nodes not in N_d U i_base.
        self.program.addConstrs(quicksum([self.ordering_flows[drone_number, arc_number] \
                                          for arc_number in params.node_number_to_outflow_arc_numbers[node_number]]) - \
                                quicksum([self.ordering_flows[drone_number, arc_number] \
                                          for arc_number in params.node_number_to_inflow_arc_numbers[node_number]]) == 0 \
                                for drone_number in params.drone_numbers \
                                for node_number in range(len(params.node_indices)) \
                                if node_number != params.base_node_number and node_number not in params.drone_node_numbers)

        # Artificial flow constraint.
        # Force artificial flow to be zero when the associated x_ij is zero.
        self.program.addConstrs(self.ordering_flows[drone_number, arc_number] <= \
                                len(params.drone_node_numbers) * 1000 * self.drone_flows[drone_number, arc_number] \
                                for drone_number in params.drone_numbers \
                                for arc_number in range(len(params.arc_indices)))

        # Coverage selection constraint.
        # Only one drone should deposit flow at a given node in N_d \ i_base.
        self.program.addConstrs(quicksum([self.coverage_selection[drone_number, node_number] for drone_number in params.drone_numbers]) \
                                == 1 for node_number in range(num_drone_nodes_not_base))

        # Coverage selection constraint.
        # The selected drone for each node should consume 1 unit of artificial flow, other drones may not.
        self.program.addConstrs(quicksum([self.ordering_flows[drone_number, arc_number] \
                                          for arc_number in params.node_number_to_outflow_arc_numbers[node_number]]) - \
                                quicksum([self.ordering_flows[drone_number, arc_number] \
                                          for arc_number in params.node_number_to_inflow_arc_numbers[node_number]]) \
                                == -self.coverage_selection[drone_number, node_number_coverage] \
                                for drone_number in params.drone_numbers \
                                for node_number_coverage, node_number in enumerate(drone_node_numbers_not_base))

        # Add range constraint for each drone.
        self.program.addConstrs(quicksum([self.drone_flows[drone_number, arc_number] * params.arc_lengths_km[arc_number] \
                                      for arc_number in range(len(params.arc_indices))]) \
                                 <= params.drone_range_km \
                                 for drone_number in params.drone_numbers)

    def solve(self) -> None:
        """"""


        # Limit solve time here for resolution 8 and 9.
        self.program.params.TimeLimit = 600  # Ten minute solve


        self.program.optimize()


def plot_cell(cell_index: H3IndexCell, color: str) -> None:
    """"""
    cell_boundary_lat_lng = h3.cell_to_boundary(cell_index)
    boundary_lats = [lat_lng[0] for lat_lng in cell_boundary_lat_lng]
    boundary_lats.append(boundary_lats[0])
    boundary_lngs = [lat_lng[1] for lat_lng in cell_boundary_lat_lng]
    boundary_lngs.append(boundary_lngs[0])
    plt.plot(boundary_lngs, boundary_lats, color=color)


def main():
    # Read json map file into geopandas data frame.
    extract_path = 'data/extracted_AA_City_Boundary'
    kml_file = os.path.join(extract_path, 'doc.kml')
    geo_data = gpd.read_file(kml_file, driver='KML')

    # Convert coordinates.
    if geo_data.crs != "EPSG:4326":
        geo_data = geo_data.to_crs("EPSG:4326")

    # Plot map boundaries.
    geo_data.boundary.plot(edgecolor="blue", linewidth=2)
    plt.axis('equal')
    plt.show()

    # Approximate map with H3 cell grid.
    map_multipolygon: MultiPolygon = geo_data.union_all()
    h3_map_polygon = geo_to_h3shape(map_multipolygon)
    hexagons = h3.h3shape_to_cells(h3_map_polygon,
                                   H3_RESOLUTION)  # 7 - 10 work well

    # Convert H3 cell grid back to geopandas data frame for visualization.
    hexagons_geo_dict = h3.cells_to_geo(hexagons)
    coordinates = hexagons_geo_dict['coordinates']
    x_coordinates = []
    y_coordinates = []
    for region_lines in coordinates:
        region_x, region_y = zip(*region_lines)
        plt.plot(region_x, region_y, color='blue')
    plt.xlabel('Longitude', fontsize=16)
    plt.ylabel('Latitude', fontsize=16)
    plt.axis('equal')
    plt.show()

    # Set up parameters for the problem.
    map_cells = hexagons
    num_drones = 1
    drone_recharge_time = 0
    drone_range = 80  # km  # 30ish works for resolution 7
    drone_speed = 100  # kph
    params = SurveillanceProgramParams(map_cells,
                                       num_drones,
                                       drone_recharge_time,
                                       drone_range,
                                       drone_speed)

    # # Set up and solve the problem.
    # model1 = MinTimeTotalCoverageSingleCircuitProgram(params)
    # model1.solve()
    #
    # # Extract optimal flow variables.
    # optimal_variables : List[gp.Var] = model1.program.getVars()
    # optimal_total_time_h = optimal_variables[-1]
    # optimal_variables_array = np.array(optimal_variables)
    # num_drone_flow_vars = len(params.drone_numbers) * len(params.arc_indices)
    # num_artificial_flow_vars = num_drone_flow_vars
    # num_min_time_vars = 1
    # optimal_drone_flow = np.array(
    #     optimal_variables[:num_drone_flow_vars]).reshape(len(params.drone_numbers), len(params.arc_indices))
    #
    # print(f'Optimal total time (hrs): {optimal_total_time_h}')
    # # print(f'Optimal traveled range (km): {sum(optimal_drone_flow)}')
    #
    # # Plot drone path in red.
    # colors = ['red', 'purple', 'orange', 'cyan', 'black']
    # for drone_number in params.drone_numbers:
    #     color = colors[drone_number]
    #     # if drone_number == 0:
    #     for arc_number in range(len(params.arc_indices)):
    #         if optimal_drone_flow[drone_number, arc_number].X > 0:
    #             origin_lat_lng = h3.cell_to_latlng(h3.get_directed_edge_origin(params.arc_indices[arc_number]))
    #             destination_lat_lng = h3.cell_to_latlng(h3.get_directed_edge_destination(params.arc_indices[arc_number]))
    #             plt.plot([origin_lat_lng[1], destination_lat_lng[1]],
    #                      [origin_lat_lng[0], destination_lat_lng[0]],
    #                      color=color)

    # # Plot all edges.
    # for arc_index in params.arc_indices:
    #     origin_lat_lng = h3.cell_to_latlng(h3.get_directed_edge_origin(arc_index))
    #     destination_lat_lng = h3.cell_to_latlng(h3.get_directed_edge_destination(arc_index))
    #     plt.plot([origin_lat_lng[1], destination_lat_lng[1]], [origin_lat_lng[0], destination_lat_lng[0]], color='green')

    # Plot all cells in blue.
    for cell_index in hexagons:
        plot_cell(cell_index, 'blue')

    # Plot drone nodes yellow.
    for drone_node_number in params.drone_node_numbers:
        cell_index = params.node_indices[drone_node_number]
        plot_cell(cell_index, 'yellow')

    # Plot start cell in green.
    cell_index = params.node_indices[params.base_node_number]
    plot_cell(cell_index, 'green')

    # Show plot.
    plt.show()

    """
    TODO:
    Is the LP relaxation tight? (If I make x_kij continuous, will they solve to integer solutions?)
        No - this maintains coverage, but causes subtours.
            Why? Question for later . . .
    
    Does it scale?
        No - not really. But the long solve has to due with getting global optimality, not the other stuff. 
    
    Does this work for multiple drones?
    
    Is the time consistent with the distance traveled?
    
    Does the range limit work?
        Yes!
    
    Does coverage work if N_d is a subset of N?
        Yes!
        
    Is the path always a circuit from different starting points?
        Yes!
    """


if __name__ == '__main__':
    main()
