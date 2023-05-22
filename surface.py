import os
import open3d as o3d
import PySimpleGUI as sg
import numpy as np
import timeit
import matplotlib.pyplot as plt


_VARS = {'window':      False,
         'popup':       False,
         'loaded':      False,
         'pcd':         False,
         'orig pcd':    False,
         'voxel ratio': 1,
         'alpha':       1,
         'bpa ratio':   1,
         'pois args':   {   'depth': 8.0,
                            'width': 0.0,
                            'scale': 1.0},
         'n_radius':    0.1,
         'n_neighbors': 10,
         'workdir':     False,
         'filepath':    False}

_STATS = {
        'bpa':  {
            'time': [],
            'args': []
        },
        'alpha':  {
            'time': [],
            'args': []
        },
        'poisson':  {
            'time': [],
            'args': []
        }}

TIMEOUT = 1

def visualize(mesh):
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    vis.add_geometry(mesh)
    vis.run()
    vis.destroy_window()

######################## WINDOW SETUP ##########################
def windowSetup():
    
    AppFont = 'Any 16'
    TextFont = 'Any 24'
    sg.theme('DarkBlack')

    pcd_frame = []
    pcd_frame.append(sg.Text('File not loaded!', key='load_text', font=TextFont))
    pcd_frame.append(sg.Button('Show PCD', font=AppFont, visible=False, key='show'))

    file_frame = []
    file_frame.append(sg.InputText(key="-FILE_PATH-"))
    file_frame.append(sg.FileBrowse(initial_folder=_VARS['workdir'], file_types=[("PCD Files", "*.pcd")], font=AppFont))
    file_frame.append(sg.Button('Load', font=AppFont))

    voxel_frame = []
    voxel_frame.append(sg.Button('Cloud recompute', font=AppFont))
    voxel_frame.append(sg.Text('Points reduction factor: ', font=TextFont))
    voxel_frame.append(sg.Slider(key='voxel_slider', range=(0.5,0.005), default_value=1, resolution=0.01,
                               orientation='h', font=AppFont, enable_events=True))
    voxel_frame.append(sg.Button('Cloud reset', font=AppFont))

    bpa_frame = []
    bpa_frame.append(sg.Button('Calculate', key='calc_bpa', font=AppFont))
    bpa_frame.append(sg.Text('Radius: ', font=TextFont))
    bpa_frame.append(sg.Slider(key='bpa_slider', range=(4.0,0.01), default_value=1, resolution=0.01,
                               orientation='h', font=AppFont, enable_events=True))

    poisson_frame = []
    poisson_frame.append(sg.Button('Calculate', key='calc_poisson', font=AppFont))
    sliders = [[sg.Slider(key='poisson_depth', range=(4,12), default_value=8,
                               orientation='h', font=AppFont, enable_events=True)],
                [sg.Slider(key='poisson_width', range=(0.0,1.0), default_value=0.0, resolution=0.01,
                               orientation='h', font=AppFont, enable_events=True, visible=False)],
                [sg.Slider(key='poisson_scale', range=(0.1,2.0), default_value=1.0, resolution=0.01,
                               orientation='h', font=AppFont, enable_events=True)]]
    
    texts = [
        [sg.Text('Depth:', font=TextFont)],
        [sg.Text('Width:', font=TextFont, visible=False)],
        [sg.Text('Scale:', font=TextFont)]
    ]
    poisson_frame.append(sg.Column(texts))
    poisson_frame.append(sg.Column(sliders))



    alpha_frame = []
    alpha_frame.append(sg.Button('Calculate', key='calc_alpha', font=AppFont))
    alpha_frame.append(sg.Text('Alpha value: ', font=TextFont))
    alpha_frame.append(sg.Slider(key='alpha_slider', range=(1.0,20.0), default_value=1, resolution=0.01,
                               orientation='h', font=AppFont, enable_events=True))
    

    n_frame = []
    n_frame.append(sg.Button('Recalculate normals', key='re_normals', font=AppFont))
    sliders_n = [[sg.Slider(key='search_radius', range=(0.01,1.0), default_value=0.1, resolution=0.01,
                               orientation='h', font=AppFont, enable_events=True)],
                [sg.Slider(key='neighbors', range=(3,100), default_value=10,
                               orientation='h', font=AppFont, enable_events=True)]]
    
    texts_n = [
        [sg.Text('Search radius:', font=TextFont)],
        [sg.Text('Max searched neighbors:', font=TextFont)]
    ]
    n_frame.append(sg.Column(texts_n))
    n_frame.append(sg.Column(sliders_n))



    cnt_reduction = [sg.Frame('Points count reduction', [voxel_frame], key='pcdr_frame', title_color='white', visible=False)]
    normals_frame = [sg.Frame('Recalculate normals of PCD', [n_frame], key='normals_frame', title_color='white', visible=False)]

    layout = [
        [sg.Frame('Point Cloud', [
                pcd_frame,
                file_frame,
                cnt_reduction,
                normals_frame],
            title_color='white')],
        [sg.Column([[sg.Text('')]], element_justification='center', expand_x=True)],
        [sg.Frame('Ball pivoting algorithm', [bpa_frame], key='bpa_frame', title_color='white', visible=False)],
        [sg.Frame('Alpha shapes algorithm', [alpha_frame], key='alpha_frame', title_color='white', visible=False)],
        [sg.Frame('Poisson algorithm', [poisson_frame], key='pois_frame', title_color='white', visible=False)],
        [sg.Button('EXIT', font=AppFont), sg.Push(), sg.Button('Show statictics', key='st_print',font=AppFont), sg.Button('Reset statistics', key='st_reset', font=AppFont)]
    ]
    _VARS['window'] = sg.Window('Surface reconstruction', layout, finalize=True)


def toggleMethods():
    _VARS['window'].find_element('pcdr_frame').Update(visible=_VARS['loaded'])
    _VARS['window'].find_element('bpa_frame').Update(visible=_VARS['loaded'])
    _VARS['window'].find_element('alpha_frame').Update(visible=_VARS['loaded'])
    _VARS['window'].find_element('pois_frame').Update(visible=_VARS['loaded'])
    _VARS['window'].find_element('normals_frame').Update(visible=_VARS['loaded'])
    _VARS['window'].find_element('show').Update(visible=_VARS['loaded'])



######################## LOADING PCD FILE ##########################
def loadPCD():

    _VARS['pcd'] = o3d.io.read_point_cloud(_VARS['filepath'], print_progress=True)
    _VARS['loaded'] = True
    calculatePCDNormals()
    _VARS['orig pcd'] = _VARS['pcd']



####################### PCD FILE ###############################
def calculatePCDNormals():
    if _VARS['loaded']:
        _VARS['pcd'].estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=_VARS['n_radius'], max_nn=int(_VARS['n_neighbors'])))
        _VARS['pcd'].orient_normals_consistent_tangent_plane(k=int(_VARS['n_neighbors']))

def cloudReset():
    _VARS['pcd'] = _VARS['orig pcd']

def cloudRecompute():
    distances = _VARS['pcd'].compute_nearest_neighbor_distance()
    avg_dist = np.mean(distances)
    _VARS['pcd'] = _VARS['orig pcd'].voxel_down_sample(_VARS['voxel ratio']*avg_dist*10)
    calculatePCDNormals()



######################## BPA ALG ##########################
def calcBPA():
    start = timeit.default_timer()
    distances = _VARS['pcd'].compute_nearest_neighbor_distance()
    avg_dist = np.mean(distances)
    radius = _VARS['bpa ratio'] * avg_dist

    bpa_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(_VARS['pcd'], o3d.utility.DoubleVector([radius, radius * 2]))

    dec_mesh = bpa_mesh.simplify_quadric_decimation(100000)

    dec_mesh.remove_degenerate_triangles()
    dec_mesh.remove_duplicated_triangles()
    dec_mesh.remove_duplicated_vertices()
    dec_mesh.remove_non_manifold_edges()

    bpa_mesh.compute_vertex_normals()

    time = timeit.default_timer() - start
    print("BPA algorithm: %8.5f sec" % (time))
    _STATS['bpa']['time'].append(time)
    _STATS['bpa']['args'].append(_VARS['bpa ratio'])
    visualize(bpa_mesh)
    
######################## POISSON ALG ##########################
def calcPoisson():
    start = timeit.default_timer()
    poisson_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(_VARS['pcd'],
                                                                             depth=int(_VARS['pois args']['depth']),
                                                                             width=_VARS['pois args']['width'],
                                                                             scale=_VARS['pois args']['scale'],
                                                                             linear_fit=False)[0]

    bbox = _VARS['pcd'].get_axis_aligned_bounding_box()
    p_mesh_crop = poisson_mesh.crop(bbox)
    p_mesh_crop.compute_vertex_normals()
    p_mesh_crop.paint_uniform_color([0.5,0.5,0.5])

    time = timeit.default_timer() - start
    print("Poisson algorithm: %8.5f sec" % (time))
    _STATS['poisson']['time'].append(time)
    _STATS['poisson']['args'].append(_VARS['pois args']['depth'])
    visualize(p_mesh_crop)

######################## ALPHA ALG ##########################
def calcAlpha():
    start = timeit.default_timer()
    distances = _VARS['pcd'].compute_nearest_neighbor_distance()
    avg_dist = np.mean(distances)

    alpha_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(_VARS['pcd'], avg_dist*_VARS['alpha'])

    alpha_dec_mesh = alpha_mesh.simplify_quadric_decimation(100000)

    alpha_dec_mesh.remove_degenerate_triangles()
    alpha_dec_mesh.remove_duplicated_triangles()
    alpha_dec_mesh.remove_duplicated_vertices()
    alpha_dec_mesh.remove_non_manifold_edges()

    alpha_mesh.compute_vertex_normals()

    time = timeit.default_timer() - start
    print("Alpha algorithm: %8.5f sec" % (time))
    _STATS['alpha']['time'].append(time)
    _STATS['alpha']['args'].append(_VARS['alpha'])
    visualize(alpha_mesh)

######################## STATICTIC OUTPUT ##########################
def printStats():
    fig, axs = plt.subplots(1,3,figsize=(9,3))
    for ax in axs:
        ax.grid(True)
    axs[0].scatter(_STATS['alpha']['args'], _STATS['alpha']['time'])
    axs[0].set_xlabel('Alpha alg time')
    axs[0].set_ylabel('Alpha value')
    axs[1].scatter(_STATS['bpa']['args'], _STATS['bpa']['time'])
    axs[1].set_xlabel('BPA alg time')
    axs[1].set_ylabel('Ball radius')
    axs[2].scatter(_STATS['poisson']['args'], _STATS['poisson']['time'])
    axs[2].set_xlabel('Poisson alg time')
    axs[2].set_ylabel('Search tree depth')
    fig.suptitle('Methods statistics')
    fig.tight_layout()
    fig.show()

######################## MAIN LOOP ##########################
def main():

    _VARS['workdir'] = os.getcwd()

    windowSetup()

    while True:
        event, values = _VARS['window'].read(timeout=TIMEOUT)
        if event == sg.WIN_CLOSED or event == 'EXIT':
            break
        
        ### PCD
        if event == 'Load':
            _VARS['filepath'] = values['-FILE_PATH-']
            loadPCD()

        if event == 'show':
            visualize(_VARS['pcd'])

        if event == 'voxel_slider':
            _VARS['voxel ratio'] = values['voxel_slider']

        if event == 'Cloud reset':
            cloudReset()

        if event == 'Cloud recompute':
            cloudRecompute()

        # BPA
        if event == 'bpa_slider':
            _VARS['bpa ratio'] = values['bpa_slider']
        if event == 'calc_bpa':
            calcBPA()

        # ALPHA
        if event == 'alpha_slider':
            _VARS['alpha'] = values['alpha_slider']
        if event == 'calc_alpha':
            calcAlpha()


        # POISSON
        if event == 'poisson_depth':
            _VARS['pois args']['depth'] = values['poisson_depth']
        if event == 'poisson_width':
            _VARS['pois args']['width'] = values['poisson_width']
        if event == 'poisson_scale':
            _VARS['pois args']['scale'] = values['poisson_scale']
        if event == 'calc_poisson':
            calcPoisson()


        # NORMALS
        if event == 'search_radius':
            _VARS['n_radius'] = values['search_radius']
        if event == 'neighbors':
            _VARS['n_neighbors'] = values['neighbors']
        if event == 're_normals':
            calculatePCDNormals()


        # STATS
        if event == 'st_print':
            print(_STATS)
            printStats()

        if event == 'st_reset':
            _STATS['alpha']['args'].clear()
            _STATS['alpha']['time'].clear()
            _STATS['bpa']['args'].clear()
            _STATS['bpa']['time'].clear()
            _STATS['poisson']['args'].clear()
            _STATS['poisson']['time'].clear()

        else:
            if _VARS['loaded']:
                _VARS['window'].find_element('load_text').Update('LOADED!')
                toggleMethods()
            else:
                _VARS['window'].find_element('load_text').Update('File not loaded!')



    _VARS['window'].close()


######################## MAIN ##########################
if __name__ == "__main__":
    main()