import defusedxml.ElementTree as ET
from defusedxml.ElementTree import SubElement
import pandas as pd
import argparse


def main():
    parser = argparse.ArgumentParser(description='Convert drive-primer log csv file to M2020 RKSML')
    
    # Positional arguments
    parser.add_argument('filename', help='Input filename')
    
    parser.add_argument('-o', '--output', help='Output file path')
        
    # Parse arguments
    args = parser.parse_args()
    
    print(f"Processing {args.filename}")
   
    output = "" 
    if args.output:
        output = args.output
    else:
        output = args.filename.replace(".csv",".rksml")
    print(f"Output to: {output}")    
        
    parse(args.filename,output)


def parse(filename, output):
    input_df = pd.read_csv(filename)
    root = ET.Element("RPK_Set", xmlns="RPK")

    sh_elem = ET.SubElement(root, "State_History", Mission="M2020", Format="SCLK", SiteType="Constant")


    rksml_map = {
        "x" : { "true_name": "ROVER_X", "units": "METERS"},
        "y" : { "true_name": "ROVER_Y", "units": "METERS"},
        "z" : { "true_name": "ROVER_Z", "units": "METERS"},
        "q_x" : { "true_name": "QUAT_X"},
        "q_y" : { "true_name": "QUAT_Y"},
        "q_z" : { "true_name": "QUAT_Z",},
        "q_w" : { "true_name": "QUAT_C"},
        "rf_s" : { "true_name": "RF_STEER", "units" : "RADIANS"},
        "rr_s" : { "true_name": "RR_STEER", "units" : "RADIANS"},
        "fl_s" : { "true_name": "LF_STEER", "units" : "RADIANS"},
        "rl_s" : { "true_name": "LR_STEER", "units" : "RADIANS"},
        "fr_d" : { "true_name": "RF_DRIVE", "units" : "RADIANS"},
        "rr_d" : { "true_name": "RR_DRIVE", "units" : "RADIANS"},
        "fl_d" : { "true_name": "LF_DRIVE", "units" : "RADIANS"},
        "lr_d" : { "true_name": "LR_DRIVE", "units" : "RADIANS"},
        "rm_d" : { "true_name": "RM_DRIVE", "units" : "RADIANS"},
        "lm_d" : { "true_name": "LM_DRIVE", "units" : "RADIANS"},
        "lb_rot" : { "true_name": "LEFT_BOGIE", "units" : "RADIANS"},
        "rb_rot" : { "true_name": "RIGHT_BOGIE", "units": "RADIANS"},
        "ld_rot" : { "true_name": "LEFT_DIFFERENTIAL", "units" : "RADIANS"},
        "rd_rot" : { "true_name": "RIGHT_DIFFERENTIAL", "units" : "RADIANS"},
    }

    ignore_cols = ["slip", "slow_slip", 
                    "wrf_x","wrf_y","wrf_z","wlf_x","wlf_y","wlf_z",
                    "wrc_x","wrc_y","wrc_z","wlc_x","wlc_y","wlc_z",
                    "wrb_x","wrb_y","wrb_z","wlb_x","wlb_y","wlb_z",    
                ]
    for _, row in input_df.iterrows():
        record = ET.SubElement(sh_elem, "Node")
        sclk = None
        for col_name, value in row.items():
            if(col_name == "m_clock"):
                sclk = str(value)
                continue
            if(col_name in ignore_cols):
                continue
            elem = SubElement(record, "Knot", Name=rksml_map[col_name]["true_name"])
            if("units" in rksml_map[col_name]):
                elem.set("Units",rksml_map[col_name]['units'])
            elem.text = str(value)
            record.set("Time", sclk)
            if(sclk == None):
                print("Error: No SCLK")
                
            
    from xml.dom import minidom
    rough_string = ET.tostring(root, encoding='utf-8')
    reparsed = minidom.parseString(rough_string)
    parsed = reparsed.toprettyxml(indent="  ", encoding='utf-8')

    with open(output, "wb") as f:
        f.write(parsed)


if __name__ == '__main__':
    main()
