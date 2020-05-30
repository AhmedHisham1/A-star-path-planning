import matplotlib.pyplot as plt
import tilemapbase
import numpy as np

def osm_plot(path):
    path = np.array(path)
    path = path[:,0:2]
    margins = [38.1598, 38.163, -122.457, -122.451]     # lats, lons
    degree_range = 0.0015

    extent = tilemapbase.Extent.from_lonlat(latitude_min=margins[0]-degree_range, latitude_max=margins[1]+degree_range,
                                            longitude_min=margins[2]-degree_range, longitude_max=margins[3]+degree_range)
    extent = extent.to_aspect(1.0)

    points = np.array([tilemapbase.project(*waypoint[::-1]) for waypoint in path])
    x, y = points[:,0], points[:,1]
    # x, y = tilemapbase.project(*goal_loc[::-1])

    fig, ax = plt.subplots(figsize=(6,6))
    plotter = tilemapbase.Plotter(extent, tile_provider=tilemapbase.tiles.build_OSM(), width=600)
    plotter.plot(ax)
    ax.scatter(x[0],y[0], marker=".", color="black", linewidth=10)
    ax.scatter(x[-1],y[-1], marker="x", color="red", linewidth=2)
    ax.plot(x,y, linewidth=2)
    ax.scatter(x,y, marker=".", color="green", linewidth=1)

    plt.xlabel('Longitude')
    plt.ylabel('Latitude')
    plt.xticks([]),plt.yticks([])
    plt.tight_layout()

    ax.grid(b=True, which='major', color='#666666', linestyle='-')
    ax.minorticks_on()
    ax.grid(b=True, which='minor', color='#999999', linestyle='-', alpha=0.2)
    plt.show()


if __name__ == "__main__":
    path = [[  38.1614402,  -122.45452764], [  38.16177789, -122.4551608 ], [  38.16219598, -122.4551608 ],
        [  38.16221206, -122.45519095], [  38.16221206, -122.45612563], [  38.16219598, -122.45615578],
        [  38.16219598, -122.45621608], [  38.1621799,  -122.45624623], [  38.1621799,  -122.45627638],
        [  38.16216382, -122.45630653], [  38.16216382, -122.45633668], [  38.16214774, -122.45636683],
        [  38.16214774, -122.45639698], [  38.16213166, -122.45642714], [  38.16213166, -122.45645729],
        [  38.16211558, -122.45648744], [  38.16211558, -122.45651759]]
    osm_plot(path)