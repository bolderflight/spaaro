"""Handle data management from Stability outputs from OpenVSP"""

# Author : Aabhash Bhandari

# REQUIRES .flt FILE FROM OPENVSP

import numpy as np
import matplotlib.pyplot as plt

# global variables
TRIM_KEYS = ['Mach_o', 'ALPHA_o', 'BETA_o', 'U_o']

BASE_KEYS = ['CLo', 'CDo', 'CYo', 'Clo', 'Cmo', 'Cno']

# DERIVATIVE_KEYS = ['CL_alpha', 'CL_beta', 'CL_mach', 'CL_p', 'CL_q', 'CL_r', 'CL_u', 'CL_alpha_2', 'CL_alpha_dot', 'CD_alpha', 'CD_beta', 'CD_mach', 'CD_p', 'CD_q', 'CD_r', 'CD_u', 'CD_alpha_2', 'CD_alpha_dot', 'CY_alpha', 'CY_beta', 'CY_mach', 'CY_p', 'CY_q', 'CY_r', 'CY_u', 'CY_alpha_2', 'CY_alpha_dot', 'Cl_alpha', 'Cl_beta', 'Cl_mach', 'Cl_p', 'Cl_q', 'Cl_r', 'Cl_u', 'Cl_alpha_2', 'Cl_alpha_dot', 'Cm_alpha', 'Cm_beta', 'Cm_mach', 'Cm_p', 'Cm_q', 'Cm_r', 'Cm_u', 'Cm_alpha_2', 'Cm_alpha_dot', 'Cn_alpha', 'Cn_beta', 'Cn_mach', 'Cn_p', 'Cn_q', 'Cn_r', 'Cn_u', 'Cn_alpha_2', 'Cn_alpha_dot']


# just selecting the derivatives that i care about
DERIVATIVE_KEYS = ['CL_alpha', 'CL_beta', 'CL_p', 'CL_q', 'CL_r', 'CL_u', 'CL_alpha_dot', 'CD_alpha', 'CD_beta', 'CD_p', 'CD_q', 'CD_r', 'CD_u',  'CD_alpha_dot', 'CY_alpha', 'CY_beta', 'CY_p', 'CY_q', 'CY_r', 'CY_u',  'CY_alpha_dot', 'Cl_alpha', 'Cl_beta', 'Cl_p', 'Cl_q', 'Cl_r', 'Cl_u',  'Cl_alpha_dot', 'Cm_alpha', 'Cm_beta', 'Cm_p', 'Cm_q', 'Cm_r', 'Cm_u',  'Cm_alpha_dot', 'Cn_alpha', 'Cn_beta', 'Cn_p', 'Cn_q', 'Cn_r', 'Cn_u',  'Cn_alpha_dot']


class StabilityCoefAndDerivatives:
    """Main Class to Handle data management from OpenVSP stability output"""

    def __init__(self, file):
        """Read the file and store data as Trim instance"""
        
        self.trim_conditions = []
        self.new_trim = None
        self.alpha = None
        self.base_coef = None
        self.derivatives = None

        with open(file, 'a') as fin:
            fin.write("###########")

        with open(file) as fin:
            lines = fin.readlines()

            for line_num, line in enumerate(lines):
                
                if line.__contains__("###########"):
                    
                    # saves the TrimPoint instance in a list attribute trim_conditions 
                    if self.new_trim is not None:
                        self.trim_conditions.append(self.new_trim)

                    # create a new TrimPoint instance every time ###### are found.
                    self.new_trim = TrimPoint()
                
                else:

                    # make sure a new TrimPoint instance is created
                    if self.new_trim is not None:

                        # handle empty lines
                        if line.strip():
                        
                            split_strings = line.split()
                            keyname = (split_strings[0]).split(':')[0]
                            val = split_strings[1]

                            # read the values and save it as dict pairs 
                            if keyname in BASE_KEYS:
                                self.new_trim.base.update({keyname: val})
                            elif keyname in DERIVATIVE_KEYS:
                                self.new_trim.derivative.update({keyname: val})
                            elif keyname in TRIM_KEYS:
                                self.new_trim.trim.update({keyname: val})
                            else:
                                pass
    
    def package_into_np_arrays(self):
        """Package the data as attributes.
        These attributes are dicts with key = coefficient / derivative name and value = numpy array for all trim points"""

        self.alpha = np.array([])
        self.base_coef = {}
        self.derivatives = {}
        first_run = True

        for trim_pts in self.trim_conditions:
            
            # storing the alpha 
            self.alpha = np.append(self.alpha, float(trim_pts.trim['ALPHA_o']))

            # going through base coefficients
            for base_key in BASE_KEYS:
                
                # creating a dict key for each base key. 
                if first_run:
                    self.base_coef[base_key] = np.array([])

                self.base_coef[base_key] = np.append(self.base_coef[base_key], float(trim_pts.base[base_key]))

            # going through derivatives 
            for derivative_key in DERIVATIVE_KEYS:

                # creating a dict key for each derivative key
                if first_run:
                    self.derivatives[derivative_key] = np.array([])

                self.derivatives[derivative_key] = np.append(self.derivatives[derivative_key], float(trim_pts.derivative[derivative_key]))
                
            first_run = False

        self.alpha = np.round(self.alpha, 2)


    def plot_base_coefs(self):
        """Plots the base coefficients"""

        fig1 = plt.figure(1)
        for base_key in BASE_KEYS:
            plt.scatter(self.alpha, self.base_coef[base_key], label=base_key)
            plt.plot(self.alpha, self.base_coef[base_key], linestyle=':')

        plt.xlabel('Alpha (deg)')
        plt.ylabel('Base coefficients')
        
        plt.title('Base Coefficients vs. Alpha')
        plt.legend()
        plt.figure(fig1.number)
        plt.show()
        

    def plot_coef_derivatives(self):
        """plotting the base coefficients"""

        fig2 = plt.figure(2)
        for derivative_key in DERIVATIVE_KEYS:
            plt.scatter(self.alpha, self.derivatives[derivative_key], label=derivative_key)
            plt.plot(self.alpha, self.derivatives[derivative_key], linestyle=':')

        plt.xlabel('Alpha (deg)')
        plt.ylabel('Coefficient derivatives')

        plt.legend()
        plt.title('Coefficients derivatives vs. Alpha')
        plt.figure(fig2.number)
        plt.show()

            
class TrimPoint:
    """Handle data storage for each trim point"""
    def __init__(self):
        self.trim = {}
        self.base = {}
        self.derivative = {}


if __name__ == "__main__":

    # USE THE .flt OUTPUT FROM OPENVSP HERE. 
    file = './session_v0_sim_model_DegenGeom.flt'

    # main instance of the class
    stab_data = StabilityCoefAndDerivatives(file)

    # Use the methods this way
    stab_data.package_into_np_arrays()
    stab_data.plot_base_coefs()
    stab_data.plot_coef_derivatives()
    

    print("base coefficient::  ", stab_data.base_coef)
    print("alpha:: ", stab_data.alpha)  
    print("derivatives:: ", stab_data.derivatives)

    print(stab_data.trim_conditions)

    print("4th trim point Cn0 base value: {}\n".format(stab_data.trim_conditions[3].base['Cno']))

    print("6th trim point Clq derivative {}".format(stab_data.trim_conditions[5].derivative['Cl_q']))

    print("last trim points alpha is {}".format(stab_data.trim_conditions[-1].trim['ALPHA_o']))
    

    # printing base coef values at 0 deg
    for base_key in BASE_KEYS:
        print('Base coef {:s} at 0 deg is {:s}\n'. format(base_key, stab_data.trim_conditions[1].base[base_key]))

    for derivative_key in DERIVATIVE_KEYS:
        print('Derivative {} at 4 deg is {}\n'.format(derivative_key, stab_data.trim_conditions[3].derivative[derivative_key]))
