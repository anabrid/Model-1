#
#  This is a demonstration program showing how to readout all computing
# elements within a Model-1 computer as described in the corresponding
# YML-file.
#
# 14-DEC-2020   B. Ulmann
#

use strict;
use warnings;

use File::Basename;
use Time::HiRes qw(usleep);
use Term::ANSIScreen;
use IO::HyCon;

$| = 1;

die "Usage: $0 <op-time in ms>\n" unless @ARGV == 1;
my ($tc) = @ARGV;

my $console = Term::ANSIScreen->new();  # This is required to rewrite the screen
my $ac = IO::HyCon->new();

my $op_time = ($tc);                    # Milliseconds, each
my $ic_time = $op_time;

$ac->reset();
$ac->set_ic_time($ic_time);
$ac->set_op_time($op_time);

$console->Cls();
while (1)
{
    $console->Cursor(0, 0);
    my $result = $ac->read_all_elements();
    my $counter = 0;
    my $last_key = '';
    for my $key (sort(keys(%$result)))
    {
        if (substr($key, 0, 2) ne $last_key) {
            $last_key = substr($key, 0, 2);
            print "\n";
            print "\n" if $counter % 4;
            $counter = 0;
        }
        print "$key: $result->{$key}{value}\t";
        print "\n" if !(++$counter % 4);
    }
    print "\nOP-TIME: $op_time ms\n";

    $ac->single_run_sync();
}
