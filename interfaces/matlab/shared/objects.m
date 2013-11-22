function [HEADER_PATHS, SRC, BIN, BINFOLDER, SRCMEX, BINMEX, BINFOLDERMEX] = objects(returnlist)
% This function returns all files to be used in the make function.
% Adapt this file whenever new files need to be compiled when updates are
% made to ACADO.
%
%  Usage:
%   >>[LOCAL_PATH_PREFIX, HEADER_PATHS, SRC, BIN, BINFOLDER, SRCMEX, BINMEX] = objects(0) :
%   Get all objects (for integrators + ocp)  
%
%   >>[LOCAL_PATH_PREFIX, HEADER_PATHS, SRC, BIN, BINFOLDER, SRCMEX, BINMEX] = objects(1) :
%   Get only objects for integrator
%
%   >>[LOCAL_PATH_PREFIX, HEADER_PATHS, SRC, BIN, BINFOLDER, SRCMEX, BINMEX] = objects(2) :
%   Get only objects for ocp
%
%   >>[LOCAL_PATH_PREFIX, HEADER_PATHS, SRC, BIN, BINFOLDER, SRCMEX, BINMEX] = objects(3) :
%   Get only objects for simulation
%
%
%  How to add new CPP files:
%   * When new files from the ACADO c++ source files needs to be added, add a new line 
%     "SRC{k} = '../../src/xxxxxx'; BIN{k} ='xxx';  BINFOLDER{k} = 'src/'; k=k+1;". 
%     Set the path in the "SRC{k}" variable, eg "src/FOLDER/FILE.cpp". Set a
%     _unique_ name for the compiled result in "BIN{k}" and set an output
%     folder for the bin file in "BINFOLDER{k}".
%
%   * When adding new MEX files (located in subdirectories of
%     <ACADOtoolkit-inst-dir>/interfaces/matlab/) add a new line at the  bottom  
%     "SRCMEX{k} = 'FOLDER/FILE.cpp'; BINMEX{k} = 'FILE'; BINFOLDERMEX{k} = 'ocp/'; k=k+1;" 
%     where "SRCMEX" is the path to the mex file, "BINMEX" is a unique name
%     and "BINFOLDERMEX" is the folder where to store the resulting mex
%     file.
%
%
%  Licence:
%    This file is part of ACADO Toolkit  - (http://www.acadotoolkit.org/)
%
%    ACADO Toolkit -- A Toolkit for Automatic Control and Dynamic Optimization.
%    Copyright (C) 2008-2009 by Boris Houska and Hans Joachim Ferreau, K.U.Leuven.
%    Developed within the Optimization in Engineering Center (OPTEC) under
%    supervision of Moritz Diehl. All rights reserved.
%
%    ACADO Toolkit is free software; you can redistribute it and/or
%    modify it under the terms of the GNU Lesser General Public
%    License as published by the Free Software Foundation; either
%    version 3 of the License, or (at your option) any later version.
%
%    ACADO Toolkit is distributed in the hope that it will be useful,
%    but WITHOUT ANY WARRANTY; without even the implied warranty of
%    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
%    Lesser General Public License for more details.
%
%    You should have received a copy of the GNU Lesser General Public
%    License along with ACADO Toolkit; if not, write to the Free Software
%    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
%
%    \author David Ariens
%    \date 2009-2010
% 

%% SETTINGS

% HEADER_PATHS = '-I../../include -I../../external_packages -I../../external_packages/include  -I../../external_packages/qpOASES-2.0/INCLUDE';
HEADER_PATHS = sprintf('-I''%s/../../include'' -I''%s/../../external_packages'' -I''%s/../../external_packages/include''  -I''%s/../../external_packages/qpOASES-2.0/INCLUDE'' -I''%s''', pwd,pwd,pwd,pwd,pwd);


%% C++ SRC ACADO OBJECTS
k = 1;

%% INTEGRATOR
SRC{k} = '../../src/utils/acado_message_handling.cpp';                BIN{k} ='acado_message_handling'; BINFOLDER{k} = 'src/'; k=k+1;
SRC{k} = '../../src/utils/acado_utils.cpp';                           BIN{k} ='acado_utils';          BINFOLDER{k} = 'src/'; k=k+1;
SRC{k} = '../../src/utils/acado_io_utils.cpp';                        BIN{k} ='acado_io_utils';       BINFOLDER{k} = 'src/'; k=k+1;
SRC{k} = '../../src/utils/acado_mat_file.cpp';                        BIN{k} ='acado_mat_file';       BINFOLDER{k} = 'src/'; k=k+1;
SRC{k} = '../../src/utils/acado_string.cpp';                          BIN{k} ='acado_string';         BINFOLDER{k} = 'src/'; k=k+1;
SRC{k} = '../../src/utils/acado_stream.cpp';                          BIN{k} ='acado_stream';         BINFOLDER{k} = 'src/'; k=k+1;

SRC{k} = '../../src/clock/clock.cpp';                                 BIN{k} ='clock';                BINFOLDER{k} = 'src/'; k=k+1;
SRC{k} = '../../src/clock/simulation_clock.cpp';                      BIN{k} ='simulation_clock';     BINFOLDER{k} = 'src/'; k=k+1;
SRC{k} = '../../src/clock/real_clock.cpp';                            BIN{k} ='real_clock';           BINFOLDER{k} = 'src/'; k=k+1;

SRC{k} = '../../src/user_interaction/options.cpp';                    BIN{k} ='options';              BINFOLDER{k} = 'src/'; k=k+1;
SRC{k} = '../../src/user_interaction/options_item.cpp';               BIN{k} ='options_item';         BINFOLDER{k} = 'src/'; k=k+1;
SRC{k} = '../../src/user_interaction/options_list.cpp';               BIN{k} ='options_list';         BINFOLDER{k} = 'src/'; k=k+1;
SRC{k} = '../../src/user_interaction/options_item_int.cpp';           BIN{k} ='options_item_int';     BINFOLDER{k} = 'src/'; k=k+1;
SRC{k} = '../../src/user_interaction/options_item_double.cpp';        BIN{k} ='options_item_double';  BINFOLDER{k} = 'src/'; k=k+1;
SRC{k} = '../../src/user_interaction/log_record_item.cpp';            BIN{k} ='log_record_item';      BINFOLDER{k} = 'src/'; k=k+1;
SRC{k} = '../../src/user_interaction/log_record.cpp';                 BIN{k} ='log_record';           BINFOLDER{k} = 'src/'; k=k+1;
SRC{k} = '../../src/user_interaction/log_collection.cpp';             BIN{k} ='log_collection';       BINFOLDER{k} = 'src/'; k=k+1;
SRC{k} = '../../src/user_interaction/logging.cpp';                    BIN{k} ='logging';              BINFOLDER{k} = 'src/'; k=k+1;
SRC{k} = '../../src/user_interaction/plot_window_subplot.cpp';        BIN{k} ='plot_window_subplot';  BINFOLDER{k} = 'src/'; k=k+1;
SRC{k} = '../../src/user_interaction/plot_window.cpp';                BIN{k} ='plot_window';          BINFOLDER{k} = 'src/'; k=k+1;
%SRC{k} = '../../external_packages/src/acado_gnuplot/gnuplot_window.cpp';   BIN{k} ='gnuplot_window';  BINFOLDER{k} = 'src/'; k=k+1;
SRC{k} = '../../src/user_interaction/plot_collection.cpp';            BIN{k} ='plot_collection';      BINFOLDER{k} = 'src/'; k=k+1;
SRC{k} = '../../src/user_interaction/plotting.cpp';                   BIN{k} ='plotting';             BINFOLDER{k} = 'src/'; k=k+1;
SRC{k} = '../../src/user_interaction/user_interaction.cpp';           BIN{k} ='user_interaction';     BINFOLDER{k} = 'src/'; k=k+1;
SRC{k} = '../../src/user_interaction/algorithmic_base.cpp';           BIN{k} ='algorithmic_base';     BINFOLDER{k} = 'src/'; k=k+1;

SRC{k} = '../../src/variables_grid/grid.cpp';                         BIN{k} ='grid';                 BINFOLDER{k} = 'src/'; k=k+1;
SRC{k} = '../../src/variables_grid/variable_settings.cpp';            BIN{k} ='variable_settings';    BINFOLDER{k} = 'src/'; k=k+1;
SRC{k} = '../../src/variables_grid/matrix_variable.cpp';              BIN{k} ='matrix_variable';      BINFOLDER{k} = 'src/'; k=k+1;
SRC{k} = '../../src/variables_grid/matrix_variables_grid.cpp';        BIN{k} ='matrix_variables_grid';BINFOLDER{k} = 'src/'; k=k+1;
SRC{k} = '../../src/variables_grid/variables_grid.cpp';               BIN{k} ='variables_grid';       BINFOLDER{k} = 'src/'; k=k+1;

SRC{k} = '../../external_packages/src/acado_csparse/acado_csparse_matlab.cpp';   BIN{k} ='acado_csparse_matlab';               BINFOLDER{k} = 'csparse/'; k=k+1;

SRC{k} = '../../src/matrix_vector/block_matrix.cpp';                  BIN{k} ='block_matrix';         BINFOLDER{k} = 'src/'; k=k+1;
SRC{k} = '../../src/matrix_vector/matrix.cpp';                        BIN{k} ='matrix';               BINFOLDER{k} = 'src/'; k=k+1;
SRC{k} = '../../src/matrix_vector/vector.cpp';                        BIN{k} ='vector';               BINFOLDER{k} = 'src/'; k=k+1;
SRC{k} = '../../src/matrix_vector/vectorspace_element.cpp';           BIN{k} ='vectorspace_element';  BINFOLDER{k} = 'src/'; k=k+1;

SRC{k} = '../../src/sparse_solver/normal_conjugate_gradient_method.cpp'; BIN{k} ='normal_conjugate_gradient_method';  BINFOLDER{k} = 'src/'; k=k+1;
SRC{k} = '../../src/sparse_solver/symmetric_conjugate_gradient_method.cpp'; BIN{k} ='symmetric_conjugate_gradient_method'; BINFOLDER{k} = 'src/'; k=k+1;
SRC{k} = '../../src/sparse_solver/conjugate_gradient_method.cpp';     BIN{k} ='conjugate_gradient_method';            BINFOLDER{k} = 'src/'; k=k+1;
SRC{k} = '../../src/sparse_solver/sparse_solver.cpp';                 BIN{k} ='sparse_solver';                        BINFOLDER{k} = 'src/'; k=k+1;

SRC{k} = '../../src/symbolic_operator/acos.cpp';                      BIN{k} ='acos';                                 BINFOLDER{k} = 'src/'; k=k+1;
SRC{k} = '../../src/symbolic_operator/addition.cpp';                  BIN{k} ='addition';                             BINFOLDER{k} = 'src/'; k=k+1;
SRC{k} = '../../src/symbolic_operator/asin.cpp';                      BIN{k} ='asin';                                 BINFOLDER{k} = 'src/'; k=k+1;
SRC{k} = '../../src/symbolic_operator/atan.cpp';                      BIN{k} ='atan';                                 BINFOLDER{k} = 'src/'; k=k+1;
SRC{k} = '../../src/symbolic_operator/binary_operator.cpp';           BIN{k} ='binary_operator';                      BINFOLDER{k} = 'src/'; k=k+1;
SRC{k} = '../../src/symbolic_operator/cos.cpp';                       BIN{k} ='cos';                                  BINFOLDER{k} = 'src/'; k=k+1;
SRC{k} = '../../src/symbolic_operator/doubleconstant.cpp';            BIN{k} ='doubleconstant';                       BINFOLDER{k} = 'src/'; k=k+1;
SRC{k} = '../../src/symbolic_operator/exp.cpp';                       BIN{k} ='exp';                                  BINFOLDER{k} = 'src/'; k=k+1;
SRC{k} = '../../src/symbolic_operator/logarithm.cpp';                 BIN{k} ='logarithm';                            BINFOLDER{k} = 'src/'; k=k+1;
SRC{k} = '../../src/symbolic_operator/operator.cpp';                  BIN{k} ='operator';                             BINFOLDER{k} = 'src/'; k=k+1;
SRC{k} = '../../src/symbolic_operator/power.cpp';                     BIN{k} ='power';                                BINFOLDER{k} = 'src/'; k=k+1;
SRC{k} = '../../src/symbolic_operator/powerint.cpp';                  BIN{k} ='powerint';                             BINFOLDER{k} = 'src/'; k=k+1;
SRC{k} = '../../src/symbolic_operator/product.cpp';                   BIN{k} ='product';                              BINFOLDER{k} = 'src/'; k=k+1;
SRC{k} = '../../src/symbolic_operator/projection.cpp';                BIN{k} ='projection';                           BINFOLDER{k} = 'src/'; k=k+1;
SRC{k} = '../../src/symbolic_operator/quotient.cpp';                  BIN{k} ='quotient';                             BINFOLDER{k} = 'src/'; k=k+1;
SRC{k} = '../../src/symbolic_operator/sin.cpp';                       BIN{k} ='sin';                                  BINFOLDER{k} = 'src/'; k=k+1;
SRC{k} = '../../src/symbolic_operator/smooth_operator.cpp';           BIN{k} ='smooth_operator';                      BINFOLDER{k} = 'src/'; k=k+1;
SRC{k} = '../../src/symbolic_operator/nonsmooth_operator.cpp';        BIN{k} ='nonsmooth_operator';                   BINFOLDER{k} = 'src/'; k=k+1;
SRC{k} = '../../src/symbolic_operator/subtraction.cpp';               BIN{k} ='subtraction';                          BINFOLDER{k} = 'src/'; k=k+1;
SRC{k} = '../../src/symbolic_operator/symbolic_index_list.cpp';       BIN{k} ='symbolic_index_list';                  BINFOLDER{k} = 'src/'; k=k+1;
SRC{k} = '../../src/symbolic_operator/tan.cpp';                       BIN{k} ='tan';                                  BINFOLDER{k} = 'src/'; k=k+1;
SRC{k} = '../../src/symbolic_operator/tree_projection.cpp';           BIN{k} ='tree_projection';                      BINFOLDER{k} = 'src/'; k=k+1;
SRC{k} = '../../src/symbolic_operator/unary_operator.cpp';            BIN{k} ='unary_operator';                       BINFOLDER{k} = 'src/'; k=k+1;


SRC{k} = '../../src/symbolic_expression/acado_syntax.cpp';            BIN{k} ='acado_syntax';                         BINFOLDER{k} = 'src/'; k=k+1;
SRC{k} = '../../src/symbolic_expression/algebraic_state.cpp';         BIN{k} ='algebraic_state';                      BINFOLDER{k} = 'src/'; k=k+1;
SRC{k} = '../../src/symbolic_expression/control.cpp';                 BIN{k} ='control';                              BINFOLDER{k} = 'src/'; k=k+1;
SRC{k} = '../../src/symbolic_expression/constraint_component.cpp';    BIN{k} ='constraint_component';                 BINFOLDER{k} = 'src/'; k=k+1;
SRC{k} = '../../src/symbolic_expression/differential_state.cpp';      BIN{k} ='differential_state';                   BINFOLDER{k} = 'src/'; k=k+1;
SRC{k} = '../../src/symbolic_expression/differential_state_derivative.cpp'; BIN{k} ='differential_state_derivative';  BINFOLDER{k} = 'src/'; k=k+1;
SRC{k} = '../../src/symbolic_expression/disturbance.cpp';             BIN{k} ='disturbance';                          BINFOLDER{k} = 'src/'; k=k+1;
SRC{k} = '../../src/symbolic_expression/expression.cpp';              BIN{k} ='expression';                           BINFOLDER{k} = 'src/'; k=k+1;
SRC{k} = '../../src/symbolic_expression/integer_control.cpp';         BIN{k} ='integer_control';                      BINFOLDER{k} = 'src/'; k=k+1;
SRC{k} = '../../src/symbolic_expression/integer_parameter.cpp';       BIN{k} ='integer_parameter';                    BINFOLDER{k} = 'src/'; k=k+1;
SRC{k} = '../../src/symbolic_expression/intermediate_state.cpp';      BIN{k} ='intermediate_state';                   BINFOLDER{k} = 'src/'; k=k+1;
SRC{k} = '../../src/symbolic_expression/output.cpp';                  BIN{k} ='output';                               BINFOLDER{k} = 'src/'; k=k+1;
SRC{k} = '../../src/symbolic_expression/parameter.cpp';               BIN{k} ='parameter';                            BINFOLDER{k} = 'src/'; k=k+1;
SRC{k} = '../../src/symbolic_expression/vt_time.cpp';                 BIN{k} ='vt_time';                              BINFOLDER{k} = 'src/'; k=k+1;

SRC{k} = '../../src/function/c_operator.cpp';                         BIN{k} ='c_operator';                           BINFOLDER{k} = 'src/'; k=k+1;
SRC{k} = '../../src/function/c_function.cpp';                         BIN{k} ='c_function';                           BINFOLDER{k} = 'src/'; k=k+1;
SRC{k} = '../../src/function/function_evaluation_tree.cpp';           BIN{k} ='function_evaluation_tree';             BINFOLDER{k} = 'src/'; k=k+1;
SRC{k} = '../../src/function/evaluation_point.cpp';                   BIN{k} ='evaluation_point';                     BINFOLDER{k} = 'src/'; k=k+1;    
SRC{k} = '../../src/function/ocp_iterate.cpp';                        BIN{k} ='ocp_iterate';                          BINFOLDER{k} = 'src/'; k=k+1;    
SRC{k} = '../../src/function/function.cpp';                           BIN{k} ='function';                             BINFOLDER{k} = 'src/'; k=k+1;
SRC{k} = '../../src/function/differential_equation.cpp';              BIN{k} ='differential_equation';                BINFOLDER{k} = 'src/'; k=k+1;
SRC{k} = '../../src/function/transition.cpp';                         BIN{k} ='transition';                           BINFOLDER{k} = 'src/'; k=k+1;
SRC{k} = '../../src/function/discretized_differential_equation.cpp';  BIN{k} ='discretized_differential_equation';    BINFOLDER{k} = 'src/'; k=k+1;
SRC{k} = '../../src/function/output_fcn.cpp';                         BIN{k} ='output_fcn';                           BINFOLDER{k} = 'src/'; k=k+1;

SRC{k} = '../../src/modeling_tools/modeling_tools.cpp';               BIN{k} ='modeling_tools';                       BINFOLDER{k} = 'src/'; k=k+1;

SRC{k} = '../../src/integrator/integrator.cpp';                       BIN{k} ='integrator';                           BINFOLDER{k} = 'src/'; k=k+1;
SRC{k} = '../../src/integrator/integrator_runge_kutta.cpp';           BIN{k} ='integrator_runge_kutta';               BINFOLDER{k} = 'src/'; k=k+1;
SRC{k} = '../../src/integrator/integrator_runge_kutta12.cpp';         BIN{k} ='integrator_runge_kutta12';             BINFOLDER{k} = 'src/'; k=k+1;
SRC{k} = '../../src/integrator/integrator_runge_kutta23.cpp';         BIN{k} ='integrator_runge_kutta23';             BINFOLDER{k} = 'src/'; k=k+1;
SRC{k} = '../../src/integrator/integrator_runge_kutta45.cpp';         BIN{k} ='integrator_runge_kutta45';             BINFOLDER{k} = 'src/'; k=k+1;
SRC{k} = '../../src/integrator/integrator_runge_kutta78.cpp';         BIN{k} ='integrator_runge_kutta78';             BINFOLDER{k} = 'src/'; k=k+1;
SRC{k} = '../../src/integrator/integrator_discretized_ode.cpp';       BIN{k} ='integrator_discretized_ode';           BINFOLDER{k} = 'src/'; k=k+1;
SRC{k} = '../../src/integrator/integrator_bdf.cpp';                   BIN{k} ='integrator_bdf';                       BINFOLDER{k} = 'src/'; k=k+1;

SRC{k} = '../../src/curve/curve.cpp';                                 BIN{k} ='curve';                                BINFOLDER{k} = 'src/'; k=k+1;


%% OPTIMAL CONTROL
if (returnlist == 0 || returnlist == 2 || returnlist == 3)
    SRC{k} = '../../src/code_generation/integrator_export.cpp';                         BIN{k} ='integrator_export';                  BINFOLDER{k} = 'src/'; k=k+1;
    SRC{k} = '../../src/code_generation/condensing_export.cpp';                         BIN{k} ='condensing_export';                  BINFOLDER{k} = 'src/'; k=k+1;
    SRC{k} = '../../src/code_generation/gauss_newton_export.cpp';                       BIN{k} ='gauss_newton_export';                BINFOLDER{k} = 'src/'; k=k+1;
    SRC{k} = '../../src/code_generation/mpc_export.cpp';                                BIN{k} ='mpc_export';                         BINFOLDER{k} = 'src/'; k=k+1;
    SRC{k} = '../../src/code_generation/export_index.cpp';                              BIN{k} ='export_index';                       BINFOLDER{k} = 'src/'; k=k+1;
    SRC{k} = '../../src/code_generation/export_data_argument.cpp';                      BIN{k} ='export_data_argument';               BINFOLDER{k} = 'src/'; k=k+1;
    SRC{k} = '../../src/code_generation/export_data.cpp';                               BIN{k} ='export_data';                        BINFOLDER{k} = 'src/'; k=k+1;
    SRC{k} = '../../src/code_generation/export_argument_list.cpp';                      BIN{k} ='export_argument_list';               BINFOLDER{k} = 'src/'; k=k+1;
    SRC{k} = '../../src/code_generation/export_function.cpp';                           BIN{k} ='export_function';                    BINFOLDER{k} = 'src/'; k=k+1;
    SRC{k} = '../../src/code_generation/export_ode_function.cpp';                       BIN{k} ='export_ode_function';                BINFOLDER{k} = 'src/'; k=k+1;
    SRC{k} = '../../src/code_generation/export_statement.cpp';                          BIN{k} ='export_statement';                   BINFOLDER{k} = 'src/'; k=k+1;
    SRC{k} = '../../src/code_generation/export_arithmetic_statement.cpp';               BIN{k} ='export_arithmetic_statement';        BINFOLDER{k} = 'src/'; k=k+1;
    SRC{k} = '../../src/code_generation/export_statement_block.cpp';                    BIN{k} ='export_statement_block';             BINFOLDER{k} = 'src/'; k=k+1;
    SRC{k} = '../../src/code_generation/export_for_loop.cpp';                           BIN{k} ='export_for_loop';                    BINFOLDER{k} = 'src/'; k=k+1;
    SRC{k} = '../../src/code_generation/export_function_call.cpp';                      BIN{k} ='export_function_call';               BINFOLDER{k} = 'src/'; k=k+1;
    SRC{k} = '../../src/code_generation/export_statement_string.cpp';                   BIN{k} ='export_statement_string';            BINFOLDER{k} = 'src/'; k=k+1;

    SRC{k} = '../../src/dynamic_system/dynamic_system.cpp';                             BIN{k} ='dynamic_system';                     BINFOLDER{k} = 'src/'; k=k+1;
     
    SRC{k} = '../../src/objective/objective_element.cpp';                               BIN{k} ='objective_element';                  BINFOLDER{k} = 'src/'; k=k+1;
    SRC{k} = '../../src/objective/lsq_term.cpp';                                        BIN{k} ='lsq_term';                           BINFOLDER{k} = 'src/'; k=k+1;
    SRC{k} = '../../src/objective/lsq_end_term.cpp';                                    BIN{k} ='lsq_end_term';                       BINFOLDER{k} = 'src/'; k=k+1;
    SRC{k} = '../../src/objective/mayer_term.cpp';                                      BIN{k} ='mayer_term';                         BINFOLDER{k} = 'src/'; k=k+1;
    SRC{k} = '../../src/objective/lagrange_term.cpp';                                   BIN{k} ='lagrange_term';                      BINFOLDER{k} = 'src/'; k=k+1;
    SRC{k} = '../../src/objective/objective.cpp';                                       BIN{k} ='objective';                          BINFOLDER{k} = 'src/'; k=k+1;

    SRC{k} = '../../src/constraint/constraint_element.cpp';                             BIN{k} ='constraint_element';                 BINFOLDER{k} = 'src/'; k=k+1;
    SRC{k} = '../../src/constraint/boundary_constraint.cpp';                            BIN{k} ='boundary_constraint';                BINFOLDER{k} = 'src/'; k=k+1;
    SRC{k} = '../../src/constraint/coupled_path_constraint.cpp';                        BIN{k} ='coupled_path_constraint';            BINFOLDER{k} = 'src/'; k=k+1;
    SRC{k} = '../../src/constraint/path_constraint.cpp';                                BIN{k} ='path_constraint';                    BINFOLDER{k} = 'src/'; k=k+1;
    SRC{k} = '../../src/constraint/algebraic_consistency_constraint.cpp';               BIN{k} ='algebraic_consistency_constraint';   BINFOLDER{k} = 'src/'; k=k+1;
    SRC{k} = '../../src/constraint/point_constraint.cpp';                               BIN{k} ='point_constraint';                   BINFOLDER{k} = 'src/'; k=k+1;
    SRC{k} = '../../src/constraint/box_constraint.cpp';                                 BIN{k} ='box_constraint';                     BINFOLDER{k} = 'src/'; k=k+1;
    SRC{k} = '../../src/constraint/constraint.cpp';                                     BIN{k} ='constraint';                         BINFOLDER{k} = 'src/'; k=k+1;

    SRC{k} = '../../src/conic_program/dense_cp.cpp';                                    BIN{k} ='dense_cp';                           BINFOLDER{k} = 'src/'; k=k+1;
    SRC{k} = '../../src/conic_program/banded_cp.cpp';                                   BIN{k} ='banded_cp';                          BINFOLDER{k} = 'src/'; k=k+1;    
    
    SRC{k} = '../../src/conic_solver/banded_cp_solver.cpp';                             BIN{k} ='banded_cp_solver';                   BINFOLDER{k} = 'src/'; k=k+1;
    SRC{k} = '../../src/conic_solver/condensing_based_cp_solver.cpp';                   BIN{k} ='condensing_based_cp_solver';         BINFOLDER{k} = 'src/'; k=k+1;
    SRC{k} = '../../src/conic_solver/dense_cp_solver.cpp';                              BIN{k} ='dense_cp_solver';                    BINFOLDER{k} = 'src/'; k=k+1;
    SRC{k} = '../../src/conic_solver/dense_qp_solver.cpp';                              BIN{k} ='dense_qp_solver';                    BINFOLDER{k} = 'src/'; k=k+1;
    
    SRC{k} = '../../src/ocp/ocp.cpp';                                                   BIN{k} ='ocp';                                BINFOLDER{k} = 'src/'; k=k+1;
    SRC{k} = '../../src/ocp/nlp.cpp';                                                   BIN{k} ='nlp';                                BINFOLDER{k} = 'src/'; k=k+1;
    SRC{k} = '../../src/ocp/multi_objective_functionality.cpp';                         BIN{k} ='multi_objective_functionality';      BINFOLDER{k} = 'src/'; k=k+1;

    SRC{k} = '../../src/dynamic_discretization/dynamic_discretization.cpp';             BIN{k} ='dynamic_discretization';             BINFOLDER{k} = 'src/'; k=k+1;
    SRC{k} = '../../src/dynamic_discretization/shooting_method.cpp';                    BIN{k} ='shooting_method';                    BINFOLDER{k} = 'src/'; k=k+1;
    SRC{k} = '../../src/dynamic_discretization/collocation_method.cpp';                 BIN{k} ='collocation_method';                 BINFOLDER{k} = 'src/'; k=k+1;
    SRC{k} = '../../src/dynamic_discretization/integration_algorithm.cpp';              BIN{k} ='integration_algorithm';              BINFOLDER{k} = 'src/'; k=k+1;

    SRC{k} = '../../external_packages/qpOASES-2.0/SRC/SQProblem.cpp';                   BIN{k} ='SQProblem';                          BINFOLDER{k} = 'qpOASES/'; k=k+1;
    SRC{k} = '../../external_packages/qpOASES-2.0/SRC/QProblem.cpp';                    BIN{k} ='QProblem';                           BINFOLDER{k} = 'qpOASES/'; k=k+1;
    SRC{k} = '../../external_packages/qpOASES-2.0/SRC/QProblemB.cpp';                   BIN{k} ='QProblemB';                          BINFOLDER{k} = 'qpOASES/'; k=k+1;
    SRC{k} = '../../external_packages/qpOASES-2.0/SRC/Bounds.cpp';                      BIN{k} ='Bounds';                             BINFOLDER{k} = 'qpOASES/'; k=k+1;
    SRC{k} = '../../external_packages/qpOASES-2.0/SRC/Constraints.cpp';                 BIN{k} ='Constraints';                        BINFOLDER{k} = 'qpOASES/'; k=k+1;
    SRC{k} = '../../external_packages/qpOASES-2.0/SRC/SubjectTo.cpp';                   BIN{k} ='SubjectTo';                          BINFOLDER{k} = 'qpOASES/'; k=k+1;
    SRC{k} = '../../external_packages/qpOASES-2.0/SRC/Indexlist.cpp';                   BIN{k} ='Indexlist';                          BINFOLDER{k} = 'qpOASES/'; k=k+1;
    SRC{k} = '../../external_packages/qpOASES-2.0/SRC/CyclingManager.cpp';              BIN{k} ='CyclingManager';                     BINFOLDER{k} = 'qpOASES/'; k=k+1;
    SRC{k} = '../../external_packages/qpOASES-2.0/SRC/Utils.cpp';                       BIN{k} ='Utils';                              BINFOLDER{k} = 'qpOASES/'; k=k+1;
    SRC{k} = '../../external_packages/qpOASES-2.0/SRC/MessageHandling.cpp';             BIN{k} ='MessageHandling';                    BINFOLDER{k} = 'qpOASES/'; k=k+1;
    SRC{k} = '../../external_packages/qpOASES-2.0/SRC/EXTRAS/SolutionAnalysis.cpp';     BIN{k} ='SolutionAnalysis';                   BINFOLDER{k} = 'qpOASES/'; k=k+1;
    SRC{k} = '../../external_packages/qpOASES-2.0/SRC/EXTRAS/OQPinterface.cpp';         BIN{k} ='OQPinterface';                       BINFOLDER{k} = 'qpOASES/'; k=k+1;



    SRC{k} = '../../src/nlp_solver/nlp_solver.cpp';                                     BIN{k} ='nlp_solver';                         BINFOLDER{k} = 'src/'; k=k+1;
    SRC{k} = '../../external_packages/src/acado_qpoases/qp_solver_qpoases.cpp';         BIN{k} ='qp_solver_qpoases';                  BINFOLDER{k} = 'src/'; k=k+1;
    SRC{k} = '../../src/nlp_solver/scp_method.cpp';                                     BIN{k} ='scp_method';                         BINFOLDER{k} = 'src/'; k=k+1;
    SRC{k} = '../../src/nlp_solver/scp_step.cpp';                                       BIN{k} ='scp_step';                           BINFOLDER{k} = 'src/'; k=k+1;
    SRC{k} = '../../src/nlp_solver/scp_step_linesearch.cpp';                            BIN{k} ='scp_step_linesearch';                BINFOLDER{k} = 'src/'; k=k+1;
    SRC{k} = '../../src/nlp_solver/scp_step_fullstep.cpp';                              BIN{k} ='scp_step_fullstep';                  BINFOLDER{k} = 'src/'; k=k+1;
    SRC{k} = '../../src/nlp_solver/scp_evaluation.cpp';                                 BIN{k} ='scp_evaluation';                     BINFOLDER{k} = 'src/'; k=k+1;
    SRC{k} = '../../src/nlp_solver/scp_merit_function.cpp';                             BIN{k} ='scp_merit_function';                 BINFOLDER{k} = 'src/'; k=k+1;
    
    SRC{k} = '../../src/nlp_derivative_approximation/nlp_derivative_approximation.cpp'; BIN{k} ='nlp_derivative_approximation';       BINFOLDER{k} = 'src/'; k=k+1;
    SRC{k} = '../../src/nlp_derivative_approximation/exact_hessian.cpp';                BIN{k} ='exact_hessian';                      BINFOLDER{k} = 'src/'; k=k+1;
    SRC{k} = '../../src/nlp_derivative_approximation/constant_hessian.cpp';             BIN{k} ='constant_hessian';                   BINFOLDER{k} = 'src/'; k=k+1;
    SRC{k} = '../../src/nlp_derivative_approximation/bfgs_update.cpp';                  BIN{k} ='bfgs_update';                        BINFOLDER{k} = 'src/'; k=k+1;
    SRC{k} = '../../src/nlp_derivative_approximation/gauss_newton_approximation.cpp';   BIN{k} ='gauss_newton_approximation';         BINFOLDER{k} = 'src/'; k=k+1;
    SRC{k} = '../../src/nlp_derivative_approximation/gauss_newton_approximation_bfgs.cpp';BIN{k} ='gauss_newton_approximation_bfgs';  BINFOLDER{k} = 'src/'; k=k+1;

    SRC{k} = '../../src/optimization_algorithm/optimization_algorithm_base.cpp';        BIN{k} ='optimization_algorithm_base';        BINFOLDER{k} = 'src/'; k=k+1;
    SRC{k} = '../../src/optimization_algorithm/optimization_algorithm.cpp';             BIN{k} ='optimization_algorithm';             BINFOLDER{k} = 'src/'; k=k+1;
    SRC{k} = '../../src/optimization_algorithm/real_time_algorithm.cpp';                BIN{k} ='real_time_algorithm';                BINFOLDER{k} = 'src/'; k=k+1;
    SRC{k} = '../../src/optimization_algorithm/parameter_estimation_algorithm.cpp';     BIN{k} ='parameter_estimation_algorithm';     BINFOLDER{k} = 'src/'; k=k+1;
    SRC{k} = '../../src/optimization_algorithm/multi_objective_algorithm.cpp';          BIN{k} ='multi_objective_algorithm';          BINFOLDER{k} = 'src/'; k=k+1;
    SRC{k} = '../../src/optimization_algorithm/weight_generation.cpp';                  BIN{k} ='weight_generation';                  BINFOLDER{k} = 'src/'; k=k+1;
    SRC{k} = '../../src/optimization_algorithm/mhe_algorithm.cpp';                      BIN{k} ='mhe_algorithm';                      BINFOLDER{k} = 'src/'; k=k+1;
end


%% Simulation
if (returnlist == 0 || returnlist == 3)  
    % curve already set in ocp
    
    SRC{k} = '../../src/controller/controller.cpp';                                     BIN{k} ='controller';                         BINFOLDER{k} = 'src/'; k=k+1;
    
    SRC{k} = '../../src/reference_trajectory/reference_trajectory.cpp';               	BIN{k} ='reference_trajectory';               BINFOLDER{k} = 'src/'; k=k+1;
    SRC{k} = '../../src/reference_trajectory/static_reference_trajectory.cpp';          BIN{k} ='static_reference_trajectory';        BINFOLDER{k} = 'src/'; k=k+1;
    SRC{k} = '../../src/reference_trajectory/periodic_reference_trajectory.cpp';        BIN{k} ='periodic_reference_trajectory';      BINFOLDER{k} = 'src/'; k=k+1;
    SRC{k} = '../../src/reference_trajectory/adaptive_reference_trajectory.cpp';        BIN{k} ='adaptive_reference_trajectory';      BINFOLDER{k} = 'src/'; k=k+1;

    SRC{k} = '../../src/estimator/estimator.cpp';                                       BIN{k} ='estimator';                          BINFOLDER{k} = 'src/'; k=k+1;
    SRC{k} = '../../src/estimator/dynamic_estimator.cpp';                               BIN{k} ='dynamic_estimator';                  BINFOLDER{k} = 'src/'; k=k+1;
    SRC{k} = '../../src/estimator/kalman_filter.cpp';                                   BIN{k} ='kalman_filter';                      BINFOLDER{k} = 'src/'; k=k+1;
    
    SRC{k} = '../../src/control_law/control_law.cpp';                                   BIN{k} ='control_law';                        BINFOLDER{k} = 'src/'; k=k+1;
    SRC{k} = '../../src/control_law/linear_state_feedback.cpp';                         BIN{k} ='linear_state_feedback';              BINFOLDER{k} = 'src/'; k=k+1;
    SRC{k} = '../../src/control_law/feedforward_law.cpp';                               BIN{k} ='feedforward_law';                    BINFOLDER{k} = 'src/'; k=k+1;
    SRC{k} = '../../src/control_law/pid_controller.cpp';                                BIN{k} ='pid_controller';                     BINFOLDER{k} = 'src/'; k=k+1;
    SRC{k} = '../../src/control_law/clipping_functionality.cpp';                        BIN{k} ='clipping_functionality';             BINFOLDER{k} = 'src/'; k=k+1;
     
    SRC{k} = '../../src/noise/noise.cpp';                                               BIN{k} ='noise';                              BINFOLDER{k} = 'src/'; k=k+1;
    SRC{k} = '../../src/noise/uniform_noise.cpp';                                       BIN{k} ='uniform_noise';                      BINFOLDER{k} = 'src/'; k=k+1;
    SRC{k} = '../../src/noise/gaussian_noise.cpp';                                      BIN{k} ='gaussian_noise';                     BINFOLDER{k} = 'src/'; k=k+1;
    SRC{k} = '../../src/noise/colored_noise.cpp';                                       BIN{k} ='colored_noise';                      BINFOLDER{k} = 'src/'; k=k+1;   
    
    SRC{k} = '../../src/transfer_device/transfer_device.cpp';                           BIN{k} ='transfer_device';                    BINFOLDER{k} = 'src/'; k=k+1;
    SRC{k} = '../../src/transfer_device/actuator.cpp';                                  BIN{k} ='actuator';                           BINFOLDER{k} = 'src/'; k=k+1;
    SRC{k} = '../../src/transfer_device/sensor.cpp';                                    BIN{k} ='sensor';                             BINFOLDER{k} = 'src/'; k=k+1;
    
    SRC{k} = '../../src/process/process.cpp';                                           BIN{k} ='process';                            BINFOLDER{k} = 'src/'; k=k+1;
        
    SRC{k} = '../../src/simulation_environment/simulation_block.cpp';                   BIN{k} ='simulation_block';                   BINFOLDER{k} = 'src/'; k=k+1;
    SRC{k} = '../../src/simulation_environment/simulation_environment.cpp';             BIN{k} ='simulation_environment';             BINFOLDER{k} = 'src/'; k=k+1;
end





%% MEX
k = 1;
%% OCP MEX
if (returnlist == 0 || returnlist == 2 || returnlist == 3)
    SRCMEX = []; BINMEX = [];BINFOLDERMEX = [];

%     SRCMEX{k} = 'examples/ocp/getting_started/gettingstarted_ACADO.cpp';     
%     BINMEX{k} = 'gettingstarted_ACADO';            
%     BINFOLDERMEX{k} = 'examples/ocp/getting_started/'; k=k+1;
%     
%     SRCMEX{k} = 'examples/ocp/invertedpendulum/pendswingup_ACADO.cpp';     
%     BINMEX{k} = 'pendswingup_ACADO';            
%     BINFOLDERMEX{k} = 'examples/ocp/invertedpendulum/'; 
%     k=k+1;

%     SRCMEX{k} = 'examples/ocp/invertedpendulum/pendswingup_ACADO.cpp';     
%     BINMEX{k} = 'pendswingup_ACADO';            
%     BINFOLDERMEX{k} = 'examples/ocp/invertedpendulum/'; 
%     k=k+1;
    %SRCMEX{k} = 'ocp/ACADOocpCALL.cpp';                 BINMEX{k} = 'ACADOocpCALL';        BINFOLDERMEX{k} = 'ocp/'; k=k+1;
end

%% INTEGRATOR MEX
if (returnlist == 0 || returnlist == 1)
    SRCMEX{k} = 'integrator/ACADOintegrators.cpp';      BINMEX{k} = 'ACADOintegrators';    BINFOLDERMEX{k} = 'integrator/'; k=k+1;
end


end


    % SRC{k} = '../../external_packages/src/acado_csparse/acado_csparse.cpp';  BIN{k} ='acado_csparse';                      BINFOLDER{k} = 'csparse/'; k=k+1;
    % SRC{k} = '../../external_packages/csparse/SRC/cs_add.c';                  BIN{k} ='cs_add';                               BINFOLDER{k} = 'csparse/'; k=k+1;
    % SRC{k} = '../../external_packages/csparse/SRC/cs_amd.c';                  BIN{k} ='cs_amd';                               BINFOLDER{k} = 'csparse/'; k=k+1;
    % SRC{k} = '../../external_packages/csparse/SRC/cs_chol.c';                 BIN{k} ='cs_chol';                              BINFOLDER{k} = 'csparse/'; k=k+1;
    % SRC{k} = '../../external_packages/csparse/SRC/cs_cholsol.c';              BIN{k} ='cs_cholsol';                           BINFOLDER{k} = 'csparse/'; k=k+1;
    % SRC{k} = '../../external_packages/csparse/SRC/cs_counts.c';               BIN{k} ='cs_counts';                            BINFOLDER{k} = 'csparse/'; k=k+1;
    % SRC{k} = '../../external_packages/csparse/SRC/cs_cumsum.c';               BIN{k} ='cs_cumsum';                            BINFOLDER{k} = 'csparse/'; k=k+1;
    % SRC{k} = '../../external_packages/csparse/SRC/cs_droptol.c';                  BIN{k} ='cs_droptol';                               BINFOLDER{k} = 'csparse/'; k=k+1;
    % SRC{k} = '../../external_packages/csparse/SRC/cs_dropzeros.c';                  BIN{k} ='cs_dropzeros';                               BINFOLDER{k} = 'csparse/'; k=k+1;
    % SRC{k} = '../../external_packages/csparse/SRC/cs_dupl.c';                 BIN{k} ='cs_dupl';                              BINFOLDER{k} = 'csparse/'; k=k+1;
    % SRC{k} = '../../external_packages/csparse/SRC/cs_entry.c';              BIN{k} ='cs_entry';                           BINFOLDER{k} = 'csparse/'; k=k+1;
    % SRC{k} = '../../external_packages/csparse/SRC/cs_etree.c';               BIN{k} ='cs_etree';                            BINFOLDER{k} = 'csparse/'; k=k+1;
    % SRC{k} = '../../external_packages/csparse/SRC/cs_fkeep.c';               BIN{k} ='cs_fkeep';                            BINFOLDER{k} = 'csparse/'; k=k+1;
    % SRC{k} = '../../external_packages/csparse/SRC/cs_gaxpy.c';                 BIN{k} ='cs_gaxpy';                              BINFOLDER{k} = 'csparse/'; k=k+1;
    % SRC{k} = '../../external_packages/csparse/SRC/cs_happly.c';              BIN{k} ='cs_happly';                           BINFOLDER{k} = 'csparse/'; k=k+1;
    % SRC{k} = '../../external_packages/csparse/SRC/cs_house.c';               BIN{k} ='cs_house';                            BINFOLDER{k} = 'csparse/'; k=k+1;
    % SRC{k} = '../../external_packages/csparse/SRC/cs_ipvec.c';               BIN{k} ='cs_ipvec';                            BINFOLDER{k} = 'csparse/'; k=k+1;
    % SRC{k} = '../../external_packages/csparse/SRC/cs_lsolve.c';               BIN{k} ='cs_lsolve';                            BINFOLDER{k} = 'csparse/'; k=k+1;
    % SRC{k} = '../../external_packages/csparse/SRC/cs_ltsolve.c';               BIN{k} ='cs_ltsolve';                            BINFOLDER{k} = 'csparse/'; k=k+1;
    % SRC{k} = '../../external_packages/csparse/SRC/cs_lu.c';                 BIN{k} ='cs_lu';                              BINFOLDER{k} = 'csparse/'; k=k+1;
    % SRC{k} = '../../external_packages/csparse/SRC/cs_lusol.c';              BIN{k} ='cs_lusol';                           BINFOLDER{k} = 'csparse/'; k=k+1;
    % SRC{k} = '../../external_packages/csparse/SRC/cs_util.c';               BIN{k} ='cs_util';                            BINFOLDER{k} = 'csparse/'; k=k+1;
    % SRC{k} = '../../external_packages/csparse/SRC/cs_multiply.c';               BIN{k} ='cs_multiply';                            BINFOLDER{k} = 'csparse/'; k=k+1;
    % SRC{k} = '../../external_packages/csparse/SRC/cs_permute.c';               BIN{k} ='cs_permute';                            BINFOLDER{k} = 'csparse/'; k=k+1;
    % SRC{k} = '../../external_packages/csparse/SRC/cs_pinv.c';               BIN{k} ='cs_pinv';                            BINFOLDER{k} = 'csparse/'; k=k+1;
    % SRC{k} = '../../external_packages/csparse/SRC/cs_post.c';                 BIN{k} ='cs_post';                              BINFOLDER{k} = 'csparse/'; k=k+1;
    % SRC{k} = '../../external_packages/csparse/SRC/cs_pvec.c';              BIN{k} ='cs_pvec';                           BINFOLDER{k} = 'csparse/'; k=k+1;
    % SRC{k} = '../../external_packages/csparse/SRC/cs_qr.c';               BIN{k} ='cs_qr';                            BINFOLDER{k} = 'csparse/'; k=k+1;
    % SRC{k} = '../../external_packages/csparse/SRC/cs_qrsol.c';               BIN{k} ='cs_qrsol';                            BINFOLDER{k} = 'csparse/'; k=k+1;
    % SRC{k} = '../../external_packages/csparse/SRC/cs_scatter.c';               BIN{k} ='cs_scatter';                            BINFOLDER{k} = 'csparse/'; k=k+1;
    % SRC{k} = '../../external_packages/csparse/SRC/cs_schol.c';               BIN{k} ='cs_schol';                            BINFOLDER{k} = 'csparse/'; k=k+1;
    % SRC{k} = '../../external_packages/csparse/SRC/cs_sqr.c';                 BIN{k} ='cs_sqr';                              BINFOLDER{k} = 'csparse/'; k=k+1;
    % SRC{k} = '../../external_packages/csparse/SRC/cs_symperm.c';              BIN{k} ='cs_symperm';                           BINFOLDER{k} = 'csparse/'; k=k+1;
    % SRC{k} = '../../external_packages/csparse/SRC/cs_tdfs.c';               BIN{k} ='cs_tdfs';                            BINFOLDER{k} = 'csparse/'; k=k+1;
    % SRC{k} = '../../external_packages/csparse/SRC/cs_malloc.c';               BIN{k} ='cs_malloc';                            BINFOLDER{k} = 'csparse/'; k=k+1;
    % SRC{k} = '../../external_packages/csparse/SRC/cs_transpose.c';               BIN{k} ='cs_transpose';                            BINFOLDER{k} = 'csparse/'; k=k+1;
    % SRC{k} = '../../external_packages/csparse/SRC/cs_compress.c';               BIN{k} ='cs_compress';                            BINFOLDER{k} = 'csparse/'; k=k+1;
    % SRC{k} = '../../external_packages/csparse/SRC/cs_usolve.c';                 BIN{k} ='cs_usolve';                              BINFOLDER{k} = 'csparse/'; k=k+1;
    % SRC{k} = '../../external_packages/csparse/SRC/cs_utsolve.c';              BIN{k} ='cs_utsolve';                           BINFOLDER{k} = 'csparse/'; k=k+1;
    % SRC{k} = '../../external_packages/csparse/SRC/cs_scc.c';               BIN{k} ='cs_scc';                            BINFOLDER{k} = 'csparse/'; k=k+1;
    % SRC{k} = '../../external_packages/csparse/SRC/cs_maxtrans.c';               BIN{k} ='cs_maxtrans';                            BINFOLDER{k} = 'csparse/'; k=k+1;
    % SRC{k} = '../../external_packages/csparse/SRC/cs_dmperm.c';               BIN{k} ='cs_dmperm';                            BINFOLDER{k} = 'csparse/'; k=k+1;
    % SRC{k} = '../../external_packages/csparse/SRC/cs_updown.c';               BIN{k} ='cs_updown';                            BINFOLDER{k} = 'csparse/'; k=k+1;
    % SRC{k} = '../../external_packages/csparse/SRC/cs_print.c';                 BIN{k} ='cs_print';                              BINFOLDER{k} = 'csparse/'; k=k+1;
    % SRC{k} = '../../external_packages/csparse/SRC/cs_norm.c';              BIN{k} ='cs_norm';                           BINFOLDER{k} = 'csparse/'; k=k+1;
    % SRC{k} = '../../external_packages/csparse/SRC/cs_load.c';               BIN{k} ='cs_load';                            BINFOLDER{k} = 'csparse/'; k=k+1;
    % SRC{k} = '../../external_packages/csparse/SRC/cs_dfs.c';               BIN{k} ='cs_dfs';                            BINFOLDER{k} = 'csparse/'; k=k+1;
    % SRC{k} = '../../external_packages/csparse/SRC/cs_reach.c';               BIN{k} ='cs_reach';                            BINFOLDER{k} = 'csparse/'; k=k+1;
    % SRC{k} = '../../external_packages/csparse/SRC/cs_spsolve.c';                 BIN{k} ='cs_spsolve';                              BINFOLDER{k} = 'csparse/'; k=k+1;
    % SRC{k} = '../../external_packages/csparse/SRC/cs_ereach.c';              BIN{k} ='cs_ereach';                           BINFOLDER{k} = 'csparse/'; k=k+1;
    % SRC{k} = '../../external_packages/csparse/SRC/cs_leaf.c';               BIN{k} ='cs_leaf';                            BINFOLDER{k} = 'csparse/'; k=k+1;
    % SRC{k} = '../../external_packages/csparse/SRC/cs_randperm.c';               BIN{k} ='cs_randperm';                            BINFOLDER{k} = 'csparse/'; k=k+1;
