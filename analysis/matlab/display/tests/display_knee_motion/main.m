model_file = 'C:\github\wisco_opensim\source\analysis\matlab\display\tests\display_knee_motion\fbknee.osim';
mot_file = 'C:\github\wisco_opensim\source\analysis\matlab\display\tests\display_knee_motion\knee_flex_40_activation_0.5_results.mot';
plugin = 'C:\github\wisco_opensim\install\plugin\WISCO_Plugin.dll';

disp = display_knee_motion(model_file,mot_file);

disp.simtk_visualizer()