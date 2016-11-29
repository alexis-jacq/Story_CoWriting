from django.shortcuts import render
#from django.http import HttpResponse


def index(request):
    #return HttpResponse("Hello, world. You're at the story index.")
    return render(request,'index.html')
# Create your views here.

def story(request):
    #return HttpResponse("Hello, world. You're at the story index.")
    return render(request,'add_story.html')
# Create your views here.

def story2(request):
    #return HttpResponse("Hello, world. You're at the story index.")
    return render(request,'add_story2.html')
# Create your views here.
def header(request):
    #return HttpResponse("Hello, world. You're at the story index.")
    return render(request,'header.html')
# Create your views here.
def footer(request):
    #return HttpResponse("Hello, world. You're at the story index.")
    return render(request,'footer.html')
# Create your views here.