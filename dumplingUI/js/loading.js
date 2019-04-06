//Item selection modal menu
var counts = new Array(3).fill(0); 
var table = 1;
$('#itemButton').click(function () {
    updateItemMenu();
});

$('#itemModal .pizza').click(function(){
    counts[0]+=1;
    updateItemMenu();
    return false;
});
$('#itemModal .coffee').click(function(){
    counts[1]+=1;
    updateItemMenu();
    return false;
});
$('#itemModal .icecream').click(function(){
    counts[2]+=1;
    updateItemMenu();
    return false;
});

$('#itemModal .clear').click(function(){
    counts[0] = 0;
    counts[1] = 0;
    counts[2] = 0;
    updateItemMenu();
});

function updateItemMenu(){
    $("#itemModal .pizza b").text(counts[0]);
    $("#itemModal .coffee b").text(counts[1]);
    $("#itemModal .icecream b").text(counts[2]);


    $("#itemButton .pizza").text(counts[0]);
    $("#itemButton .coffee").text(counts[1]);
    $("#itemButton .icecream").text(counts[2]);

    if(counts[0] != 0 || counts[1] != 0 || counts[2] !=0){
        $("#add").removeClass("disabled");
    }
    else{
        $("#add").addClass("disabled");
    }
};

function clearCounts(){
    counts[0] = 0;
    counts[1] = 0;
    counts[2] = 0;
    updateItemMenu();
};

//Table selection modal menu
$('#tableButton').click(function () {
    updateTableMenu();
});

$('#t1').click(function(){
    table=1;
    updateTableMenu();
});
$('#t2').click(function(){
    table=2;
    updateTableMenu();
});
$('#t3').click(function(){
    table=3;
    updateTableMenu();
});
$('#t4').click(function(){
    table=4;
    updateTableMenu();
});
$('#t5').click(function(){
    table=5;
    updateTableMenu();
});
function updateTableMenu(){
    $("#tableButton span").text(table);
};

//Add job
$('#add').click(function(){
    clearCounts();
});