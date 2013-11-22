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
%qpOASES solves (a series) of convex quadratic programmming (QP)
%problems of the following form:
%
%                min   0.5*x'Hx + x'g
%                s.t.  lb  <=  x <= ub
%                      lbA <= Ax <= ubA  [optional]
%
%Call
%    [obj,x,y,status,nWSRout] = qpOASES( H,g,A,lb,ub,lbA,ubA,{nWSR,{x0}} )
%for solving the above-mentioned QP performing at most nWSR working set
%recalculations and starting from an initial guess x0. H must be a symmetric
%and positive definite matrix and all vectors g, lb, ub, lbA, ubA have to be
%given as column vectors. If nWSR is not specified, a default value is chosen.
%If no initial guess is provided, iterations start from the origin.
%
%Call
%     [obj,x,y,status,nWSRout] = qpOASES( H,g,lb,ub,{nWSR,{x0}} )
%for solving the above-mentioned QP without constraints.
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
%If not a single QP but a sequence of QPs with varying vectors is to be solved,
%the i-th QP is given by the i-th columns of the QP vectors g, lb, ub, lbA, ubA
%(i.e. they are matrices in this case). Both matrices H and A remain constant.
%
%See also QPOASES_SEQUENCE, QPOASES_SEQUENCESB, QPOASES_SEQUENCEVM
%
%
%For additional information see the qpOASES User's Manual or
%visit http://www.qpOASES.org/.
%
%Please send remarks and questions to support@qpOASES.org!