# * --------------------------------------------------------------------------
# *  CppADCodeGen: C++ Algorithmic Differentiation with Source Code Generation:
# *    Copyright (C) 2018 Ciengis
# *
# *  CppADCodeGen is distributed under multiple licenses:
# *
# *   - Eclipse Public License Version 1.0 (EPL1), and
# *   - GNU General Public License Version 3 (GPL3).
# *
# *  EPL1 terms and conditions can be found in the file "epl-v10.txt", while
# *  terms and conditions for the GPL3 can be found in the file "gpl3.txt".
# * ----------------------------------------------------------------------------
# * Author: Joao Leal

import gdb


def is_null(value_ptr):
    if value_ptr.type.code == gdb.TYPE_CODE_PTR:
        value_str = str(value_ptr)
        return value_str == '0x0' or value_str == 'NULL'
    else:
        return False


def get_smart_ptr_val(smart_ptr):
    pointer = smart_ptr['_M_t']['_M_t']['_M_head_impl']
    if is_null(pointer):
        return None
    else:
        return pointer.dereference()


class CGPrettyPrinter:
    def __init__(self, val):
        self.val = val

    def to_string(self):
        try:
            node_ptr = self.val['node_']
            if not is_null(node_ptr):
                node = node_ptr.dereference()

                out = "variable CG ("

                name = get_smart_ptr_val(node['name_'])
                if name is not None:
                    out += "'" + str(name) + "', "
                out += str(node['operation_'])

                value = get_smart_ptr_val(self.val['value_'])
                if value is not None:
                    out += ", " + str(value)

                out += ")"
                return out
            else:
                value = get_smart_ptr_val(self.val['value_'])
                return "constant CG (" + str(value) + ")"

        except Exception as e:
            print('An exception occurred: {}'.format(e))

        return out

    def children(self):
        c = []

        # value
        value = get_smart_ptr_val(self.val['value_'])
        if value is not None:
            c.append(('value_', value))

        # node
        node_ptr = self.val['node_']
        if not is_null(node_ptr):
            c.append(('node_', node_ptr.dereference()))

        return c

    def display_hint(self):
        if is_null(self.val['node_']):
            return 'string'
        else:
            return 'map'

###############################################################################
#
#  Create and register the pretty printers
#  ---------------------------------------
#
# Register the printers in gdb using the gdb.printing module
#
###############################################################################


def build_pretty_printer():
    pp = gdb.printing.RegexpCollectionPrettyPrinter("CppAD::cg")
    pp.add_printer('CppAD::cg::CG', '^CppAD::cg::CG<.*>$', CGPrettyPrinter)
    return pp


def reload():
    # Remove the pretty printer if it exists
    for printer in gdb.pretty_printers:
        if printer.name == 'CppAD::cg':
            gdb.pretty_printers.remove(printer)
            break

    # Create the new pretty printer
    gdb.printing.register_pretty_printer(gdb.current_objfile(), build_pretty_printer())


reload()
