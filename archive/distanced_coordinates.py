import math
import matplotlib.pyplot as plt

def read_coordinates(file_path):
    coordinates = []
    with open(file_path, 'r') as file:
        for line in file:
            if line.startswith("Filtered Coordinates:"):
                parts = line.split(',')
                x = int(parts[0].split('=')[1])
                y = int(parts[1].split('=')[1])
                coordinates.append((x, y))
    return coordinates

def calculate_distance(coord1, coord2):
    return math.sqrt((coord2[0] - coord1[0]) ** 2 + (coord2[1] - coord1[1]) ** 2)

def filter_coordinates(coordinates, min_distance):
    filtered_coords = [coordinates[0]]
    for coord in coordinates[1:]:
        if calculate_distance(filtered_coords[-1], coord) >= min_distance:
            filtered_coords.append(coord)
    return filtered_coords

def plot_coordinates(coordinates):
    x_vals = [coord[0] for coord in coordinates]
    y_vals = [coord[1] for coord in coordinates]
    plt.scatter(x_vals, y_vals)
    plt.xlabel('X Coordinate')
    plt.ylabel('Y Coordinate')
    plt.title('Filtered Coordinates')
    plt.xlim(0, 1023)
    plt.ylim(0, 1023)
    plt.grid(True)
    plt.show()

def main():
    file_path = '/home/eli/Desktop/My_Projects/car/coordinates.txt'
    output_file_path = '/home/eli/Desktop/My_Projects/car/distanced_coordinates.txt'
    min_distance = 50
    coordinates = read_coordinates(file_path)
    filtered_coordinates = filter_coordinates(coordinates, min_distance)
    
    with open(output_file_path, 'w') as output_file:
        for coord in filtered_coordinates:
            output_file.write(f"{{{coord[0]}, {coord[1]}}},\n")
    
    plot_coordinates(filtered_coordinates)

if __name__ == "__main__":
    main()
