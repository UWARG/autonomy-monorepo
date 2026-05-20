""" 
Takes a MAVLink message, and converts it into a JSON string format that can be sent to the frontend. 
Establishes a clear, and expected format for the frontend to parse. 

Should be in the format of {type, payload}, where type informs the type of message, and later which widget the 
string is directed towards. 
"""

