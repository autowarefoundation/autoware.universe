#!/usr/bin/perl

$use_sim_time_str='<param name="use_sim_time" value="$(env AW_ROS2_USE_SIM_TIME false)"/>' . "\n";

$out_line="";
while(<>){
	if(/^(\s*)<\s*node\s/){
		$indent=$1;
		$out_line = $indent . "  " . $use_sim_time_str;
		if(/\/\s*>\s*$/){
			$re_str = quotemeta($&);
			~s/$re_str/ >\n/;
			print;
			$out_line = $out_line . $indent . "</node>\n";
			print $out_line;
			$out_line = "";
		}else{
			print;
		}
	}elsif($out_line ne ""){
		if(/^\s*<\s*param\s*name.*use_sim_time.*AW_ROS2_USE_SIM_TIME/){
			$out_line = "";
		}elsif(/^\s*<\s*\/\s*node\s*>\s*$/){
			print $out_line;
			$out_line = "";
		}
		print;
	}else{
		print;
	}
}
