%qpOASES -- An Implementation of the Online Active Set Strategy.
%Copyright (C) 2007-2009 by Hans Joachim Ferreau et al. All rights reserved.
%
%qpOASES is distributed under the terms of the
%GNU Lesser General Public License 2.1 in the hope that it will be
%useful, but WITHOUT ANY WARRANTY; without even the implied warranty
%of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
%See the GNU Lesser General Public License for more details.
%
%---------------------------------------------------------------------------------
%
%qpOASES_sequenceSB is intended to solve a sequence of simply bounded
%convex quadratic programmming (QP) problems of the following form:
%
%                min   0.5*x'Hx + x'g
%                s.t.  lb  <=  x <= ub
%
%I) Call
%     [obj,x,y,status,nWSRout] = ...
%                      qpOASES_sequenceSB( 'i',H,g,lb,ub,{nWSR,{x0}} )
%for initialising and solving the first above-mentioned QP of the sequence
%performing at most nWSR working set recalculations and starting from an
%initial guess x0. H must be a symmetric and positive definite matrix and
%all vectors g, lb, ub have to be given as column vectors. If nWSR is not
%specified, a default value is chosen. If no initial guess is provided,
%iterations start from the origin.
%
%II) Call
%     [obj,x,y,status,nWSRout] = ...
%                      qpOASES_sequenceSB( 'h',g,lb,ub,{nWSR} )
%for hotstarting from the previous QP solution to the one of the next QP
%given by the vectors g, lb, ub. If nWSR is not specified, a default value
%is chosen.
%
%III) Having solved the last QP of your sequence, call
%     qpOASES_sequenceSB( 'c' )
%in order to cleanup the internal memory.
%
%
%Optional Outputs (only obj is mandatory):
%    obj     -  optimal objective function value (if status==0)
%    x       -  optimal primal solution vector   (if status==0)
%    y       -  optimal dual solution vector     (if status==0)
%    status  -   0: QP solved
%                1: maximum number of working set recalculations reached
%               -1: QP could not be solved
%    nWSRout -  number of working set recalculations actually performed
%
%
%See also QPOASES_SEQUENCE, QPOASES_SEQUENCEVM
%
%
%For additional information see the qpOASES User's Manual or
%visit http://www.qpOASES.org/.
%
%Please send remarks and questions to support@qpOASES.org!