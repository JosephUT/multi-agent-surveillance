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

# Resolution for H3 cells (0-15).
H3_RESOLUTION = 8

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
        self.drone_node_numbers: List[int] = [i for i in range(len(self.node_indices))]  # TODO: Assign via cloud generation.

        # The index of the node where all drones must start and finish their circuits.
        self.base_node_index: H3IndexCell = h3.latlng_to_cell(ANN_ARBOR_PD_LAT_LNG[0],
                                                              ANN_ARBOR_PD_LAT_LNG[1],
                                                              H3_RESOLUTION)

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


class MinTimeTotalCoverageSingleCircuitProgram:
    def __init__(self, params: SurveillanceProgramParams) -> None:
        """"""
        self.program = gp.Model("MinTimeTotalCoverageSingleCircuitProgram")

        # Add decision variable drone_flows where drone_flows[drone_number][arc_number] is the flow of drone
        # drone_number across arc arc_number.
        self.drone_flows = self.program.addVars(len(params.drone_numbers), len(params.arc_indices),
                                                lb=0,
                                                vtype=GRB.INTEGER)

        # Add continuous decision variable to represent the max circuit time for any single drone.
        self.max_drone_circuit_time_h = self.program.addVar(vtype=GRB.CONTINUOUS,
                                                          name="max_drone_circuit_time_h")

        # Minimize the max circuit time for any single drone.
        self.program.setObjective(self.max_drone_circuit_time_h, GRB.MINIMIZE)

        # Upper bound the circuit time of each drone with the max circuit time variable.

        self.program.addConstrs(1/params.drone_speed_kph * \
                                 quicksum([self.drone_flows[drone_number, arc_number] \
                                           for arc_number in range(len(params.arc_indices))]) \
                                 <= self.max_drone_circuit_time_h for drone_number in params.drone_numbers)

        # Add coverage constraint for all nodes that must be observed by a drone.
        self.program.addConstrs(quicksum([self.drone_flows[drone_number, arc_number] \
                                     for drone_number in params.drone_numbers \
                                     for arc_number in params.node_number_to_inflow_arc_numbers[node_number]]) >= 1 \
                                    for node_number in params.drone_node_numbers)

        # Add flow constraints for all drones for all nodes in the graph.
        self.program.addConstrs(quicksum([self.drone_flows[drone_number, out_arc_number] \
                                     for out_arc_number in params.node_number_to_outflow_arc_numbers[node_number]]) - \
                                quicksum([self.drone_flows[drone_number, in_arc_number] \
                                     for in_arc_number in params.node_number_to_inflow_arc_numbers[node_number]]) == 0 \
                                for drone_number in params.drone_numbers \
                                for node_number in range(len(params.node_indices)))

        # Add single circuit constraint for each drone.
        self.program.addConstrs(quicksum([self.drone_flows[drone_number, arc_number] \
                                     for arc_number in params.node_number_to_outflow_arc_numbers[params.base_node_number]]) <= 1 \
                                for drone_number in params.drone_numbers)

        # Add range constraint for each drone.
        self.program.addConstrs(quicksum([self.drone_flows[drone_number, arc_number] * params.arc_lengths_km[arc_number] \
                                      for arc_number in range(len(params.arc_indices))]) \
                                 <= params.drone_range_km \
                                 for drone_number in params.drone_numbers)


    def solve(self) -> None:
        """"""
        self.program.optimize()


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
    drone_range = 100
    drone_speed = 10
    params = SurveillanceProgramParams(map_cells,
                                       num_drones,
                                       drone_recharge_time,
                                       drone_range,
                                       drone_speed)

    # Set up and solve the problem.
    model1 = MinTimeTotalCoverageSingleCircuitProgram(params)
    model1.solve()

    # Extract optimal flow variables.
    optimal_variables : List[gp.Var] = model1.program.getVars()
    max_drone_circuit_time_h = optimal_variables[-1]
    optimal_drone_flow = np.array(optimal_variables[:-1]).reshape(len(params.drone_numbers), len(params.arc_indices))

    # Plot lines between center points of each cell for each drone.
    for drone_number in params.drone_numbers:
        for arc_number in range(len(params.arc_indices)):
            if optimal_drone_flow[drone_number, arc_number].X > 0:
                origin_lat_lng = h3.cell_to_latlng(h3.get_directed_edge_origin(params.arc_indices[arc_number]))
                destination_lat_lng = h3.cell_to_latlng(h3.get_directed_edge_destination(params.arc_indices[arc_number]))
                plt.plot([origin_lat_lng[1], destination_lat_lng[1]],
                         [origin_lat_lng[0], destination_lat_lng[0]],
                         color='red')

    # # Plot all edges.
    # for arc_index in params.arc_indices:
    #     origin_lat_lng = h3.cell_to_latlng(h3.get_directed_edge_origin(arc_index))
    #     destination_lat_lng = h3.cell_to_latlng(h3.get_directed_edge_destination(arc_index))
    #     plt.plot([origin_lat_lng[1], destination_lat_lng[1]], [origin_lat_lng[0], destination_lat_lng[0]], color='green')

    # Plot all cells
    for cell_index in hexagons:
        cell_boundary_lat_lng = h3.cell_to_boundary(cell_index)
        boundary_lats = [lat_lng[0] for lat_lng in cell_boundary_lat_lng]
        boundary_lats.append(boundary_lats[0])
        boundary_lngs = [lat_lng[1] for lat_lng in cell_boundary_lat_lng]
        boundary_lngs.append(boundary_lngs[0])
        plt.plot(boundary_lngs, boundary_lats, color='blue')
    # plt.plot(path_lngs, path_lats, color='red')
    plt.show()

    # Plot the nodes selected


if __name__ == '__main__':
    main()
