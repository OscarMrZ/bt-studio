import os
from django.conf import settings
from rest_framework.decorators import api_view
from rest_framework.response import Response
from .serializers import FileContentSerializer
from . import app_generator
from . import tree_generator
from . import json_translator
from django.http import HttpResponse
from django.http import JsonResponse
import mimetypes
import json
import base64
import zipfile
from io import BytesIO


@api_view(['GET'])
def create_project(request):

    project_name = request.GET.get('project_name')
    folder_path = os.path.join(settings.BASE_DIR, 'filesystem')
    project_path = os.path.join(folder_path, project_name)
    action_path = os.path.join(project_path, 'actions')

    if not os.path.exists(project_path):
        os.mkdir(project_path)
        os.mkdir(action_path)
        return Response({'success': True})
    else:
        return Response({'success': False, 'message': 'Project already exists'}, status=400)


@api_view(['GET'])
def get_project_list(request):

    folder_path = os.path.join(settings.BASE_DIR, 'filesystem')

    try:
        # List all folders in the directory
        project_list = [d for d in os.listdir(
            folder_path) if os.path.isdir(os.path.join(folder_path, d))]

        # Return the list of projects
        return Response({'project_list': project_list})

    except Exception as e:
        return Response({'error': f'An error occurred: {str(e)}'}, status=500)


@api_view(['POST'])
def save_project(request):

    # Get the app name and the graph
    project_name = request.data.get('project_name')
    graph_json = request.data.get('graph_json')

    # Generate the paths
    base_path = os.path.join(settings.BASE_DIR, 'filesystem')
    project_path = os.path.join(base_path, project_name)
    graph_path = os.path.join(project_path, "graph.json")

    if project_path and graph_json:

        try:
            # Obtain pretty json
            graph = json.loads(graph_json)
            graph_formated = json.dumps(graph, indent=4)

            with open(graph_path, 'w') as f:
                f.write(graph_formated)

            return JsonResponse({'success': True})

        except Exception as e:
            return JsonResponse({'success': False, 'message': f'Error deleting file: {str(e)}'}, status=500)
    else:
        return Response({'error': 'app_name parameter is missing'}, status=400)


@api_view(['GET'])
def get_project_graph(request):

    project_name = request.GET.get('project_name')

    # Generate the paths
    base_path = os.path.join(settings.BASE_DIR, 'filesystem')
    project_path = os.path.join(base_path, project_name)
    graph_path = os.path.join(project_path, "graph.json")

    # Check if the project exists
    if os.path.exists(graph_path):
        try:
            with open(graph_path, 'r') as f:
                graph_data = json.load(f)
            return JsonResponse({'success': True, 'graph_json': graph_data})
        except Exception as e:
            return JsonResponse({'success': False, 'message': f'Error reading file: {str(e)}'}, status=500)
    else:
        return Response({'error': 'The project does not have a graph definition'}, status=404)

@api_view(['GET'])
def get_simulation_zip_base64(request):
    
    # Get project_name from the request
    project_name = request.query_params.get('project_name')
    if not project_name:
        return Response({'success': False, 'message': 'Project name is required.'}, status=400)

    # Define the path to the simulation folder
    simulation_folder = os.path.join(settings.BASE_DIR, 'filesystem', project_name, 'sim')

    # Check if the simulation folder exists
    if not os.path.exists(simulation_folder):
        return Response({'success': False, 'message': f'Simulation folder for project "{project_name}" not found.'}, status=404)

    # Create a zip file in memory
    in_memory_zip = BytesIO()
    with zipfile.ZipFile(in_memory_zip, mode='w', compression=zipfile.ZIP_DEFLATED) as zipf:
        for root, dirs, files in os.walk(simulation_folder):
            for file in files:
                file_path = os.path.join(root, file)
                # Store the file under its name only, not including the 'simulation' folder in the path
                zipf.write(file_path, os.path.relpath(file_path, simulation_folder))

    # Encode the zip file in base64
    in_memory_zip.seek(0)
    encoded_zip = base64.b64encode(in_memory_zip.read()).decode()

    return Response({'success': True, 'base64_zip': encoded_zip})

@api_view(['GET'])
def get_manager_base64(request):

    # Define the path to the simulation folder
    simulation_folder = os.path.join(settings.BASE_DIR, 'bt-manager')

    # Check if the simulation folder exists
    if not os.path.exists(simulation_folder):
        return Response({'success': False, 'message': f'Manager folder not found'}, status=404)

    # Create a zip file in memory
    in_memory_zip = BytesIO()
    with zipfile.ZipFile(in_memory_zip, mode='w', compression=zipfile.ZIP_DEFLATED) as zipf:
        for root, dirs, files in os.walk(simulation_folder):
            for file in files:
                file_path = os.path.join(root, file)
                # Store the file under its name only, not including the 'simulation' folder in the path
                zipf.write(file_path, os.path.relpath(file_path, simulation_folder))

    # Encode the zip file in base64
    in_memory_zip.seek(0)
    encoded_zip = base64.b64encode(in_memory_zip.read()).decode()

    return Response({'success': True, 'base64_zip': encoded_zip})

@api_view(['GET'])
def get_file_list(request):

    project_name = request.GET.get('project_name')
    folder_path = os.path.join(settings.BASE_DIR, 'filesystem')
    project_path = os.path.join(folder_path, project_name)
    action_path = os.path.join(project_path, 'actions')

    try:
        # List all files in the directory
        file_list = [f for f in os.listdir(
            action_path) if os.path.isfile(os.path.join(action_path, f))]

        # Return the list of files
        return Response({'file_list': file_list})

    except Exception as e:
        return Response({'error': f'An error occurred: {str(e)}'}, status=500)


@api_view(['GET'])
def get_file(request):

    project_name = request.GET.get('project_name', None)
    filename = request.GET.get('filename', None)

    # Make folder path relative to Django app
    folder_path = os.path.join(settings.BASE_DIR, 'filesystem')
    project_path = os.path.join(folder_path, project_name)
    action_path = os.path.join(project_path, 'actions')

    if filename:
        file_path = os.path.join(action_path, filename)
        if os.path.exists(file_path):
            with open(file_path, 'r') as f:
                content = f.read()
            serializer = FileContentSerializer({'content': content})
            return Response(serializer.data)
        else:
            return Response({'error': 'File not found'}, status=404)
    else:
        return Response({'error': 'Filename parameter is missing'}, status=400)


@api_view(['GET'])
def create_file(request):

    # Get the file info
    project_name = request.GET.get('project_name', None)
    filename = request.GET.get('filename', None)

    # Make folder path relative to Django app
    folder_path = os.path.join(settings.BASE_DIR, 'filesystem')
    project_path = os.path.join(folder_path, project_name)
    action_path = os.path.join(project_path, 'actions')
    file_path = os.path.join(action_path, filename)

    if not os.path.exists(file_path):
        with open(file_path, 'w') as f:
            f.write('')  # Empty content
        return Response({'success': True})
    else:
        return Response({'success': False, 'message': 'File already exists'}, status=400)


@api_view(['GET'])
def delete_file(request):

    # Get the file info
    project_name = request.GET.get('project_name', None)
    filename = request.GET.get('filename', None)

    # Make folder path relative to Django app
    folder_path = os.path.join(settings.BASE_DIR, 'filesystem')
    project_path = os.path.join(folder_path, project_name)
    action_path = os.path.join(project_path, 'actions')
    file_path = os.path.join(action_path, filename)

    if os.path.exists(file_path):
        try:
            os.remove(file_path)
            return JsonResponse({'success': True})
        except Exception as e:
            return JsonResponse({'success': False, 'message': f'Error deleting file: {str(e)}'}, status=500)
    else:
        return JsonResponse({'success': False, 'message': 'File does not exist'}, status=404)


@api_view(['POST'])
def save_file(request):

    project_name = request.data.get('project_name')
    filename = request.data.get('filename')
    content = request.data.get('content')

    folder_path = os.path.join(settings.BASE_DIR, 'filesystem')
    project_path = os.path.join(folder_path, project_name)
    action_path = os.path.join(project_path, 'actions')
    file_path = os.path.join(action_path, filename)

    try:
        with open(file_path, 'w') as f:
            f.write(content)
        return Response({'success': True})
    except Exception as e:
        return Response({'success': False, 'message': str(e)}, status=400)


@api_view(['POST'])
def translate_json(request):

    folder_path = os.path.join(settings.BASE_DIR, 'filesystem')

    try:
        content = request.data.get('content')
        if content is None:
            return Response({'success': False, 'message': 'Content is missing'}, status=400)

        # Pass the JSON content to the translate function
        json_translator.translate(content, folder_path + "/tree.xml")

        return Response({'success': True})
    except Exception as e:
        return Response({'success': False, 'message': str(e)}, status=400)


@api_view(['POST'])
def generate_app(request):

    # Get the app name
    app_name = request.data.get('app_name')
    content = request.data.get('content')

    # Make folder path relative to Django app
    base_path = os.path.join(settings.BASE_DIR, 'filesystem')
    project_path = os.path.join(base_path, app_name)
    action_path = os.path.join(project_path, 'actions')
    tree_path = os.path.join('/tmp/tree.xml')
    self_contained_tree_path = os.path.join('/tmp/self_contained_tree.xml')
    template_path = os.path.join(settings.BASE_DIR, 'ros_template')
    tree_gardener_src = os.path.join(settings.BASE_DIR, 'tree_gardener')

    if app_name and content:

        try:
            # Generate a basic tree from the JSON definition
            json_translator.translate(content, tree_path)

            # Generate a self-contained tree
            tree_generator.generate(
                tree_path, action_path, self_contained_tree_path)

            # Using the self-contained tree, package the ROS 2 app
            zip_file_path = app_generator.generate(
                self_contained_tree_path, app_name, template_path, action_path, tree_gardener_src)

            # Confirm ZIP file exists
            if not os.path.exists(zip_file_path):
                return Response({'success': False, 'message': 'ZIP file not found'}, status=400)

            # Prepare file response
            zip_file = open(zip_file_path, 'rb')
            mime_type, _ = mimetypes.guess_type(zip_file_path)
            response = HttpResponse(zip_file, content_type=mime_type)
            response[
                'Content-Disposition'] = f'attachment; filename={os.path.basename(zip_file_path)}'

            return response

        except Exception as e:
            return Response({'success': False, 'message': str(e)}, status=400)
    else:
        return Response({'error': 'app_name parameter is missing'}, status=500)
