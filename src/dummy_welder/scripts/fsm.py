###
#   Python implementation of Mealy FSM interface
#   Copyright (C) 2019 by Pablo Costas Franco
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 3 of the License, or
#   (at your option) any later version.
#   This program is distributed in the hope that it will be useful,
#   but WITHOUT ANY WARRANTY; without even the implied warranty of
#   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#   GNU General Public License for more details.
#   You should have received a copy of the GNU General Public License
#   along with this program.  If not, see <http://www.gnu.org/licenses/>
# 
#
###


class fsm:
    def __init__(self, trans_table, init_state):
        self.current_state = init_state
        self.trans_table   = trans_table

    def fire(self):
        for trans in self.trans_table:
            if(self.current_state == trans.orig_state) and (trans.cond()):
                self.current_state = trans.dest_state
                if trans.trans_callback is not None:    
                    trans.trans_callback() 
                break


class fsm_trans:
    def __init__(self, orig, cond, dest, callback): 
        self.orig_state = orig
        self.cond = cond
        self.dest_state = dest
        self.trans_callback = callback
