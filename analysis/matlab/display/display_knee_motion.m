%% Display Motion
%==========================================================================
classdef display_knee_motion
    properties
        mot_file
        model_file
        
        mot
        model
        nTimeStep
       
    end
    methods
        function obj = display_knee_motion(model_file,mot_file,plugin_file)
            import org.opensim.modeling.*
            opensimCommon.LoadOpenSimLibraryExact(plugin_file);
            obj.mot_file = mot_file;
            obj.model_file = model_file;
            
            obj.model = Model(model_file);
            
            [obj.mot.data, obj.mot.hdr, obj.mot.labels] = read_opensim_mot(mot_file);
            obj.nTimeStep = size(obj.mot.data,1);
            
        end
        
        function write_vtk_files(settings_file)
            analyzeTool = AnalyzeTool(settings_file);
            analyzeTool.run();
        end
        
        function simtk_visualizer(obj)
            obj.model.setUseVisualizer(true);
            state = obj.model.initSystem();

            initState = State(state);


            sviz = obj.model.updVisualizer().updSimbodyVisualizer();
            sviz.setShowSimTime(true);
            % Show "ground and sky" background instead of just a black background.
            sviz.setBackgroundTypeByInt(1);

            % Show help text in the visualization window.
            help = DecorativeText('Press any key to start a new simulation; ESC to quit.');
            help.setIsScreenText(true);
            sviz.addDecoration(0, Transform(Vec3(0, 0, 0)), help);

            obj.model.getVisualizer().show(initState);

            % Wait for the user to hit a key before starting the simulation.
            silo = obj.model.updVisualizer().updInputSilo();
            
            %Prescribe Coordinates
            coord_set = obj.model.getCoordinateSet();
            
            for j = 1:coord_set.getSize()
                coord = coord_set.get(i);
                    
                ind = find(strcmp(coord.getName(),obj.mot.labels));

                if(isempty(ind))
                    coord.set_locked(true);
                else
                    coord.set_prescribed(true)

                    spline = GCVSpline();
                    spline.setDegree(5);

                    for k = 1:obj.nTimeStep 
                        spline.addPoint(obj.mot.data(k,1),obj.mot.data(k,ind));
                    end
                    
                    coord.set_prescried_function(spline);
                end
            end
            
            
            compList = obj.model.getComponentsList();
            compIter = compList.begin();
            while ~compIter.equals(compList.end())
                if ~isempty(strfind(compIter.getConcreteClassName(), ...
                        'Coordinate'))
                    comp = obj.model.getComponent(compIter.getAbsolutePathName());
                    coord = Coordinate.safeDownCast(comp);
                    
                    coord.set_prescribed(true);
                    coord.set_prescribed_function();
                end
                compIter.next();
            end
                
            while true

                    % Ignore any previous key presses.
                    silo.clear();
                    % Get the next key press.
                    while ~silo.isAnyUserInput()
                        pause(0.01);
                    end
                    % The alternative `waitForKeyHit()` is not ideal for MATLAB, as MATLAB
                    % is not able to interrupt native functions, and `waitForKeyHit()` will
                    % hang if the simbody-visualizer is killed.
                    key = silo.takeKeyHitKeyOnly();
                    % Key 27 is ESC; see the SimTK::Visualizer::InputListener::KeyCode enum.
                    if key == 27
                        sviz.shutdown();
                        return;
                    end

                % Clear the table for all TableReporters. Note: this does not handle
                % TableReporters for Vec3s, etc.
                compList = model.getComponentsList();
                compIter = compList.begin();
                while ~compIter.equals(compList.end())
                    if ~isempty(strfind(compIter.getConcreteClassName(), ...
                            'TableReporter__double_'))
                        comp = model.getComponent(compIter.getAbsolutePathName());
                        reporter = TableReporter.safeDownCast(comp);
                        reporter.clearTable();
                    end
                    compIter.next();
                end

                % Simulate.
                state = State(initState);
                manager = Manager(obj.model);
                manager.integrate(state, 5.0);
                


            end
        end
    end

end
            