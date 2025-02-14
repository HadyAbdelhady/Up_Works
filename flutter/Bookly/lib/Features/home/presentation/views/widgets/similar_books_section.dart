import 'package:bookly/core/utils/styles.dart';
import 'package:flutter/material.dart';

import 'similar_books_list_view.dart';

class SimilarBooksSection extends StatelessWidget {
  const SimilarBooksSection({super.key});

  @override
  Widget build(BuildContext context) {
    return const Column(
      crossAxisAlignment: CrossAxisAlignment.start,
      children: [
        Padding(
          padding: EdgeInsets.only(left: 18),
          child: Text(
            'You can also like',
            style: Styles.textStyle18,
          ),
        ),
        SimilarBooksListView(),
        SizedBox(
          height: 20,
        )
      ],
    );
  }
}
