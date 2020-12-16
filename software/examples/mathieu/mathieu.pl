#
#  This little example program varies the parameter a/10 in the setup of
# Mathieu's equation (cf. "Analog and Hybrid Computer Programming", DeGruyter,
# 2020, pp. 116 ff.) and records the resulting wave form of the variable y
# for each distinct value of a/10. 
#  It is assumed that a/10 is controlled by potentiometer 0 of the HC module.
#
# 16-DEC-2020   B. Ulmann
#

use strict;
use warnings;

use IO::HyCon;
use Data::Dumper;

my $ac = IO::HyCon->new();
$ac->setup();

for my $a (0 .. 200) {
    print "Running with a = $a...\n";
    $ac->set_pt('a', $a / 200);
    $ac->digital_output(0, 1);
    $ac->digital_output(0, 0);
    $ac->single_run_sync(); # Run and collect data as defined in the ro-group
#    $ac->get_data();
#    $ac->plot();
}
