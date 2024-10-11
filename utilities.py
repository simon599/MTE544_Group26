from math import atan2, asin, sqrt

M_PI=3.1415926535

class Logger:
    def __init__(self, filename, headers=["e", "e_dot", "e_int", "stamp"]):
        self.filename = filename

        with open(self.filename, 'w') as file:
            header_str=""

            for header in headers:
                header_str+=header
                header_str+=", "
            
            header_str+="\n"
            
            file.write(header_str)


    def log_values(self, values_list):

        with open(self.filename, 'a') as file:
            vals_str=""

            # TODO Part 5: Write the values from the list to the file
            # Log values with comma separation.
            vals_str += ", ".join(map(str, values_list))
            vals_str += "\n"
            
            file.write(vals_str)
            

    def save_log(self):
        pass

class FileReader:
    def __init__(self, filename):
        
        self.filename = filename
        
        
    def read_file(self):
        
        read_headers=False

        table=[]
        headers=[]
        with open(self.filename, 'r') as file:
            # Skip the header line

            if not read_headers:
                for line in file:
                    values=line.strip().split(',')

                    for val in values:
                        if val=='':
                            break
                        headers.append(val.strip())

                    read_headers=True
                    break
            
            next(file)
            
            # Read each line and extract values
            for line in file:
                values = line.strip().split(',')
                
                row=[]                
                
                for val in values:
                    if val=='':
                        break
                    row.append(float(val.strip()))

                table.append(row)
        
        return headers, table


# TODO Part 5: Implement the conversion from Quaternion to Euler Angles
def euler_from_quaternion(quat):
    """
    Convert quaternion (w in last place) to euler roll, pitch, yaw.
    quat = [x, y, z, w]
    """
    # Unpack components of the given (unit) quaternion.
    x = quat.x
    y = quat.y
    z = quat.z
    w = quat.w

    # roll = rot about x, pitch = rot about y, yaw = rot about z
    # Calculate yaw using this formula
    roll = atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x**2 + y**2))
    pitch = asin(2.0 * (w * y - x * z))
    yaw = atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y**2 + z**2))

    # just unpack yaw
    return yaw


