var ptpv2 = require('ptpv2');

ptpv2.init('10.42.0.1', 0, function(){
	var synced = ptpv2.is_synced();
	var ptpMaster = ptpv2.ptp_master();
	var time = ptpv2.ptp_time();

	console.log(synced, ptpMaster, time);
});